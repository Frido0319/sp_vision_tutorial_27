#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iostream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

namespace
{

class GimbalCommandWatchdog
{
public:
  explicit GimbalCommandWatchdog(io::Gimbal & gimbal)
  : gimbal_(gimbal), heartbeat_ns_(now_ns()), thread_(&GimbalCommandWatchdog::run, this)
  {
  }

  ~GimbalCommandWatchdog()
  {
    stop_.store(true, std::memory_order_release);
    if (thread_.joinable()) thread_.join();

    std::lock_guard<std::mutex> lock(send_mutex_);
    gimbal_.send(false, false, 0.0F, 0.0F);
  }

  bool send(bool control, float yaw, float pitch)
  {
    std::lock_guard<std::mutex> lock(send_mutex_);
    if (control && (!std::isfinite(yaw) || !std::isfinite(pitch))) {
      gimbal_.send(false, false, 0.0F, 0.0F);
      return false;
    }

    heartbeat_ns_.store(now_ns(), std::memory_order_release);
    gimbal_.send(control, false, yaw, pitch);
    return control;
  }

private:
  static constexpr auto kCommandTimeout = 250ms;
  static constexpr auto kPollInterval = 50ms;

  static std::int64_t now_ns()
  {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
  }

  void run()
  {
    const auto timeout_ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(kCommandTimeout).count();

    while (!stop_.load(std::memory_order_acquire)) {
      std::this_thread::sleep_for(kPollInterval);
      if (stop_.load(std::memory_order_acquire)) break;

      const auto heartbeat_ns = heartbeat_ns_.load(std::memory_order_acquire);
      if (now_ns() - heartbeat_ns < timeout_ns) continue;

      std::lock_guard<std::mutex> lock(send_mutex_);
      if (stop_.load(std::memory_order_acquire)) break;
      if (now_ns() - heartbeat_ns_.load(std::memory_order_acquire) >= timeout_ns) {
        gimbal_.send(false, false, 0.0F, 0.0F);
      }
    }
  }

  io::Gimbal & gimbal_;
  std::atomic<std::int64_t> heartbeat_ns_;
  std::atomic<bool> stop_{false};
  std::mutex send_mutex_;
  std::thread thread_;
};

int run_task_1(const std::string & config_path)
{
  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;  // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);
  GimbalCommandWatchdog command_watchdog(gimbal);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  while (!exiter.exit()) {
    camera.read(img, t);
    q = gimbal.q(t);
    solver.set_R_gimbal2world(q);
    auto armors = yolo.detect(img);
    for (auto & armor : armors) {
      solver.solve(armor);
    }

    bool control = gimbal.mode() == io::GimbalMode::AUTO_AIM && !armors.empty();
    float command_yaw = 0.0F;
    float command_pitch = 0.0F;
    float target_distance = 0.0F;
    if (control) {
      const auto & target = armors.front();
      command_yaw = static_cast<float>(target.ypd_in_world[0]);
      command_pitch = static_cast<float>(target.ypd_in_world[1]);
      target_distance = static_cast<float>(target.ypd_in_world[2]);
    }

    control = command_watchdog.send(control, command_yaw, command_pitch);
    if (!control) {
      command_yaw = 0.0F;
      command_pitch = 0.0F;
      target_distance = 0.0F;
    } else if (!std::isfinite(target_distance)) {
      target_distance = 0.0F;
    }

    plotter.plot(
      {{"control", control ? 1 : 0},
       {"command_yaw", command_yaw},
       {"command_pitch", command_pitch},
       {"target_distance", target_distance}});

    int key = cv::waitKey(1);
    if (key == 'q' || key == 27) break;
  }

  return 0;
}

}  // namespace

int main(int argc, char * argv[])
{
  try {
    cv::CommandLineParser cli(argc, argv, keys);
    auto config_path = cli.get<std::string>("@config-path");
    if (cli.has("help") || !cli.has("@config-path")) {
      cli.printMessage();
      return 0;
    }

    return run_task_1(config_path);
  } catch (const std::exception & e) {
    std::cerr << "Task 1 failed: " << e.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "Task 1 failed: unknown exception\n";
    return 1;
  }
}
