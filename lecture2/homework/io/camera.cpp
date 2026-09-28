#include "camera.hpp"

#include <yaml-cpp/yaml.h>

#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "hikrobot/include/MvCameraControl.h"

namespace
{

std::runtime_error sdk_error(const std::string & operation, int code)
{
  std::ostringstream message;
  message << operation << " failed (Hikrobot SDK error 0x" << std::hex << std::uppercase << code
          << ')';
  return std::runtime_error(message.str());
}

void require_ok(int code, const std::string & operation)
{
  if (code != MV_OK) throw sdk_error(operation, code);
}

unsigned int parse_hex_id(const std::string & text, const std::string & key)
{
  std::size_t used = 0;
  unsigned long value = 0;
  try {
    value = std::stoul(text, &used, 16);
  } catch (const std::exception &) {
    throw std::runtime_error("Invalid hexadecimal " + key + ": " + text);
  }
  if (used != text.size() || value > 0xFFFF) {
    throw std::runtime_error("Invalid hexadecimal " + key + ": " + text);
  }
  return static_cast<unsigned int>(value);
}

std::pair<unsigned int, unsigned int> parse_vid_pid(const std::string & text)
{
  const auto separator = text.find(':');
  if (separator == std::string::npos || text.find(':', separator + 1) != std::string::npos) {
    throw std::runtime_error("vid_pid must use four-digit hexadecimal VID:PID format");
  }
  return {
    parse_hex_id(text.substr(0, separator), "VID"),
    parse_hex_id(text.substr(separator + 1), "PID")};
}

MV_CC_DEVICE_INFO * select_usb_device(
  const MV_CC_DEVICE_INFO_LIST & devices, unsigned int vid, unsigned int pid)
{
  for (unsigned int index = 0; index < devices.nDeviceNum; ++index) {
    auto * device = devices.pDeviceInfo[index];
    if (device == nullptr || device->nTLayerType != MV_USB_DEVICE) continue;
    const auto & usb = device->SpecialInfo.stUsb3VInfo;
    if (usb.idVendor == vid && usb.idProduct == pid) return device;
  }
  return nullptr;
}

class FrameBufferGuard
{
public:
  FrameBufferGuard(void * handle, MV_FRAME_OUT & frame) : handle_(handle), frame_(frame) {}
  ~FrameBufferGuard() { MV_CC_FreeImageBuffer(handle_, &frame_); }

  FrameBufferGuard(const FrameBufferGuard &) = delete;
  FrameBufferGuard & operator=(const FrameBufferGuard &) = delete;

private:
  void * handle_;
  MV_FRAME_OUT & frame_;
};

}  // namespace

namespace io
{

Camera::Camera(const std::string & config_path)
{
  try {
    const auto config = YAML::LoadFile(config_path);
    const auto camera_name = config["camera_name"].as<std::string>();
    const auto exposure_ms = config["exposure_ms"].as<double>();
    const auto gain = config["gain"].as<double>();
    const auto frame_rate = config["frame_rate"].as<double>();
    const auto vid_pid = config["vid_pid"].as<std::string>();
    timeout_ms_ = config["timeout_ms"].as<int>();

    if (camera_name != "hikrobot") {
      throw std::runtime_error("Only camera_name 'hikrobot' is supported");
    }
    if (exposure_ms <= 0 || gain < 0 || frame_rate <= 0 || timeout_ms_ <= 0) {
      throw std::runtime_error("Camera exposure, gain, frame rate, and timeout are out of range");
    }

    const auto [vid, pid] = parse_vid_pid(vid_pid);
    MV_CC_DEVICE_INFO_LIST devices{};
    require_ok(MV_CC_EnumDevices(MV_USB_DEVICE, &devices), "MV_CC_EnumDevices");
    auto * selected = select_usb_device(devices, vid, pid);
    if (selected == nullptr) {
      std::ostringstream message;
      message << "No Hikrobot USB camera matched " << vid_pid << " (found " << devices.nDeviceNum
              << " USB camera(s))";
      throw std::runtime_error(message.str());
    }

    require_ok(MV_CC_CreateHandle(&handle_, selected), "MV_CC_CreateHandle");
    require_ok(MV_CC_OpenDevice(handle_), "MV_CC_OpenDevice");
    require_ok(
      MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF),
      "disable trigger mode for continuous acquisition");
    require_ok(
      MV_CC_SetGrabStrategy(handle_, MV_GrabStrategy_LatestImagesOnly),
      "select latest-frame grab strategy");
    require_ok(
      MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS),
      "set continuous white balance");
    require_ok(
      MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF),
      "disable automatic exposure");
    require_ok(
      MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF), "disable automatic gain");
    require_ok(
      MV_CC_SetFloatValue(handle_, "ExposureTime", exposure_ms * 1000.0), "set exposure time");
    require_ok(MV_CC_SetFloatValue(handle_, "Gain", gain), "set gain");
    require_ok(MV_CC_SetFrameRate(handle_, static_cast<float>(frame_rate)), "set frame rate");
    require_ok(MV_CC_StartGrabbing(handle_), "MV_CC_StartGrabbing");
    grabbing_ = true;
  } catch (const YAML::Exception & error) {
    cleanup();
    throw std::runtime_error("Invalid camera config '" + config_path + "': " + error.what());
  } catch (...) {
    cleanup();
    throw;
  }
}

Camera::~Camera() { cleanup(); }

void Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  MV_FRAME_OUT raw{};
  require_ok(
    MV_CC_GetImageBuffer(handle_, &raw, static_cast<unsigned int>(timeout_ms_)),
    "MV_CC_GetImageBuffer");
  FrameBufferGuard frame_guard(handle_, raw);
  timestamp = std::chrono::steady_clock::now();

  if (raw.pBufAddr == nullptr || raw.stFrameInfo.nWidth == 0 || raw.stFrameInfo.nHeight == 0) {
    throw std::runtime_error("Hikrobot returned an empty frame");
  }

  cv::Mat converted(
    static_cast<int>(raw.stFrameInfo.nHeight), static_cast<int>(raw.stFrameInfo.nWidth), CV_8UC3);
  MV_CC_PIXEL_CONVERT_PARAM parameters{};
  parameters.nWidth = raw.stFrameInfo.nWidth;
  parameters.nHeight = raw.stFrameInfo.nHeight;
  parameters.pSrcData = raw.pBufAddr;
  parameters.nSrcDataLen = raw.stFrameInfo.nFrameLen;
  parameters.enSrcPixelType = raw.stFrameInfo.enPixelType;
  parameters.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
  parameters.pDstBuffer = converted.data;
  parameters.nDstBufferSize = static_cast<unsigned int>(converted.total() * converted.elemSize());
  require_ok(MV_CC_ConvertPixelType(handle_, &parameters), "MV_CC_ConvertPixelType");

  img = std::move(converted);
}

void Camera::cleanup() noexcept
{
  if (handle_ == nullptr) return;
  if (grabbing_) {
    MV_CC_StopGrabbing(handle_);
    grabbing_ = false;
  }
  MV_CC_CloseDevice(handle_);
  MV_CC_DestroyHandle(handle_);
  handle_ = nullptr;
}

}  // namespace io
