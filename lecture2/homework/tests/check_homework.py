#!/usr/bin/env python3
"""Static acceptance checks for Lecture 2 homework source files."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


class CameraContractTest(unittest.TestCase):
    def test_public_api_contains_only_required_three_functions(self) -> None:
        header = read("io/camera.hpp")
        match = re.search(r"class\s+Camera\s*\{\s*public:(.*?)private:", header, re.S)
        self.assertIsNotNone(match, "Camera must have explicit public/private sections")
        public = match.group(1)
        declarations = [
            line.strip()
            for line in public.splitlines()
            if line.strip() and not line.strip().startswith("//")
        ]
        self.assertEqual(
            declarations,
            [
                "explicit Camera(const std::string & config_path);",
                "~Camera();",
                "void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);",
            ],
        )

    def test_private_data_members_end_with_underscore(self) -> None:
        header = read("io/camera.hpp")
        self.assertIn("private:", header, "Camera must declare private state")
        private = header.split("private:", 1)[1].rsplit("};", 1)[0]
        data_lines = [
            line.strip()
            for line in private.splitlines()
            if re.match(r"^(?:void\s*\*|bool|int|double|std::string)\s+\w+", line.strip())
            and "(" not in line
        ]
        self.assertTrue(data_lines, "Camera must declare private state")
        for line in data_lines:
            name = re.search(r"([A-Za-z_]\w*)\s*(?:\{|=|;)", line)
            self.assertIsNotNone(name, line)
            self.assertTrue(name.group(1).endswith("_"), line)

    def test_direct_sdk_lifecycle_is_implemented(self) -> None:
        source = read("io/camera.cpp")
        required = [
            "MV_CC_EnumDevices",
            "MV_CC_CreateHandle",
            "MV_CC_OpenDevice",
            "MV_CC_StartGrabbing",
            "MV_CC_GetImageBuffer",
            "MV_CC_FreeImageBuffer",
            "MV_CC_StopGrabbing",
            "MV_CC_CloseDevice",
            "MV_CC_DestroyHandle",
        ]
        for function in required:
            self.assertIn(function, source)
        self.assertNotIn('#include "io/hikrobot/hikrobot.hpp"', source)


class PipelineSourceTest(unittest.TestCase):
    def test_yolo_pipeline_meets_required_visualization(self) -> None:
        source = read("main.cpp")
        for token in [
            "io::Camera",
            "auto_aim::YOLO",
            ".read(",
            ".detect(",
            "tools::draw_points",
            "tools::draw_text",
            "cv::Scalar(0, 255, 0)",
            "cv::imshow",
            "cv::waitKey(1)",
            "auto_aim::COLORS",
            "auto_aim::ARMOR_NAMES",
        ]:
            self.assertIn(token, source)
        self.assertRegex(source, r"key\s*==\s*'q'.*key\s*==\s*27")

    def test_apriltag_pipeline_is_completed(self) -> None:
        source = read("opencv.cpp")
        for token in [
            "io::Camera",
            "auto_charge::AprilTagDetector",
            ".read(",
            ".detect(",
            "tools::draw_points",
            "tools::draw_text",
            "cv::Scalar(0, 255, 0)",
            "cv::imshow",
            "cv::waitKey(1)",
        ]:
            self.assertIn(token, source)
        self.assertRegex(source, r"key\s*==\s*'q'.*key\s*==\s*27")


class SmokeTestSourceTest(unittest.TestCase):
    def test_cmake_exposes_opt_in_model_smoke_target(self) -> None:
        cmake = read("CMakeLists.txt")
        self.assertIn("option(BUILD_HOMEWORK_TESTS", cmake)
        self.assertIn("add_executable(model_smoke tests/model_smoke.cpp)", cmake)
        self.assertIn("add_test(NAME model_smoke", cmake)

    def test_model_smoke_covers_yolo_and_apriltag(self) -> None:
        self.assertTrue((ROOT / "tests/model_smoke.cpp").is_file(), "model_smoke.cpp is missing")
        source = read("tests/model_smoke.cpp")
        for token in [
            "auto_aim::YOLO",
            ".detect(",
            "cv::aruco::drawMarker",
            "auto_charge::AprilTagDetector",
            "tools::draw_points",
            "cv::imwrite",
        ]:
            self.assertIn(token, source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
