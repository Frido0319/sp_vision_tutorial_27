#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <exception>
#include <iostream>
#include <string>

#include "tasks/apriltag_detector.hpp"
#include "tasks/yolo.hpp"
#include "tools/img_tools.hpp"

int main(int argc, char ** argv)
{
  try {
    cv::Mat blank(640, 640, CV_8UC3, cv::Scalar::all(0));
    auto_aim::YOLO yolo("./configs/yolo.yaml", false);
    const auto armors = yolo.detect(blank, 0);

    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Mat marker;
    cv::aruco::drawMarker(dictionary, 10, 300, marker, 1);
    cv::Mat scene_gray(500, 500, CV_8UC1, cv::Scalar::all(255));
    marker.copyTo(scene_gray(cv::Rect(100, 100, marker.cols, marker.rows)));
    cv::Mat scene;
    cv::cvtColor(scene_gray, scene, cv::COLOR_GRAY2BGR);

    auto_charge::AprilTagDetector detector("./configs/yolo.yaml");
    const auto tags = detector.detect(scene);
    const auto expected = std::find_if(tags.begin(), tags.end(), [](const auto & tag) {
      return tag.id == 10 && tag.corners.size() == 4;
    });
    if (expected == tags.end()) {
      std::cerr << "AprilTag smoke test did not detect target ID 10\n";
      return 1;
    }

    const auto green = cv::Scalar(0, 255, 0);
    tools::draw_points(scene, expected->corners, green, 2);
    tools::draw_text(scene, "AprilTag 10", expected->center, green, 0.7, 2);
    const std::string output = argc > 1 ? argv[1] : "./tests/apriltag_smoke.png";
    if (!cv::imwrite(output, scene)) {
      std::cerr << "Could not write smoke-test visualization to " << output << '\n';
      return 1;
    }

    std::cout << "PASS: YOLO loaded and processed a blank frame (" << armors.size()
              << " detections); AprilTag ID 10 detected and rendered to " << output << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "Model smoke test failed: " << error.what() << '\n';
    return 1;
  }
}
