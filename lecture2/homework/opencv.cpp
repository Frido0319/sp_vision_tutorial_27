#include "io/camera.hpp"
#include "opencv2/opencv.hpp"
#include "tasks/apriltag_detector.hpp"
#include "tools/img_tools.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <string>

int main()
{
  try {
    io::Camera camera("./configs/camera.yaml");
    auto_charge::AprilTagDetector detector("./configs/yolo.yaml");

    while (true) {
      cv::Mat img;
      std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);

      const auto detections = detector.detect(img);
      for (const auto & detection : detections) {
        const auto green = cv::Scalar(0, 255, 0);
        tools::draw_points(img, detection.corners, green, 2);
        tools::draw_text(
          img, "AprilTag " + std::to_string(detection.id), detection.center, green, 0.7, 2);
      }

      cv::imshow("OpenCV AprilTag detection", img);
      const int key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 27) break;
    }
  } catch (const std::exception & error) {
    std::cerr << "Lecture 2 optional homework failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
