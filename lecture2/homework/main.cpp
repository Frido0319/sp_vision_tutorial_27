#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <string>

int main()
{
  try {
    io::Camera camera("./configs/camera.yaml");
    auto_aim::YOLO yolo("./configs/yolo.yaml", false);
    int frame_count = 0;

    while (true) {
      cv::Mat img;
      std::chrono::steady_clock::time_point timestamp;
      camera.read(img, timestamp);

      const auto armors = yolo.detect(img, frame_count++);
      for (const auto & armor : armors) {
        const auto green = cv::Scalar(0, 255, 0);
        tools::draw_points(img, armor.points, green, 2);
        const auto label = auto_aim::COLORS.at(armor.color) + " " +
                           auto_aim::ARMOR_NAMES.at(armor.name);
        tools::draw_text(img, label, armor.center, green, 0.7, 2);
      }

      cv::imshow("armor detection", img);
      const int key = cv::waitKey(1) & 0xFF;
      if (key == 'q' || key == 27) break;
    }
  } catch (const std::exception & error) {
    std::cerr << "Lecture 2 homework failed: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
