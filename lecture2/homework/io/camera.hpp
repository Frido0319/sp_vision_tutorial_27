#ifndef IO__CAMERA_HPP
#define IO__CAMERA_HPP

#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>

namespace io
{

class Camera
{
public:
  explicit Camera(const std::string & config_path);
  ~Camera();
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  void cleanup() noexcept;

  void * handle_{nullptr};
  bool grabbing_{false};
  int timeout_ms_{100};
};

}  // namespace io

#endif  // IO__CAMERA_HPP
