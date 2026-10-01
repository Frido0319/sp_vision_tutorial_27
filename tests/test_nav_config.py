#!/usr/bin/env python3

from pathlib import Path
import unittest

import yaml


CONFIG_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "sp_nav_bringup"
    / "config"
    / "nav_params.yaml"
)

EXPECTED = {
    ("planner_server", "costmap_topic"): "/global_costmap",
    ("planner_server", "local_costmap_topic"): "/local_costmap/costmap",
    ("planner_server", "path_topic"): "/global_path",
    ("planner_server", "plugin_name"): "AStar",
    ("planner_server", "plugin_type"): "sp_global_planner/AStarPlanner",
    ("controller_server", "local_path_topic"): "/global_path",
    ("controller_server", "odom_topic"): "/Odometry",
    ("controller_server", "odom_frame_id"): "lidar_odom",
    ("controller_server", "map_frame_id"): "map",
    ("controller_server", "cmd_vel_topic"): "/sentry/cmd_vel",
    ("controller_server", "base_frame_id"): "base_link",
    ("controller_server", "plugin_name"): "PidController",
    ("controller_server", "plugin_type"): "PidController",
}

CONTROLLER_KEYS = {
    "kp",
    "ki",
    "kd",
    "integral_limit",
    "lookahead_distance",
    "max_speed",
    "max_acceleration",
    "corner_slowdown_angle",
    "corner_speed_ratio",
    "goal_slowdown_distance",
    "goal_tolerance",
}


class NavigationConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with CONFIG_PATH.open(encoding="utf-8") as stream:
            cls.config = yaml.safe_load(stream)

    def params(self, node):
        return self.config[node]["ros__parameters"]

    def test_required_topics_frames_and_plugins(self):
        for (node, key), expected in EXPECTED.items():
            with self.subTest(node=node, key=key):
                value = self.params(node).get(key)
                self.assertNotIn(value, (None, ""))
                self.assertEqual(value, expected)

    def test_map_and_control_rate(self):
        map_params = self.params("esdf_map_publisher")
        self.assertEqual(map_params.get("map_yaml"), "sp_nav_bringup/map/maze_map.yaml")
        self.assertGreater(float(map_params.get("publish_rate_hz", 0.0)), 0.0)
        self.assertGreater(float(self.params("controller_server").get("control_frequency", 0.0)), 0.0)

    def test_planner_and_controller_parameters_are_complete(self):
        planner = self.params("planner_server").get("AStar", {})
        self.assertEqual(planner.get("lethal_cost"), 90)
        self.assertGreater(float(planner.get("cost_weight", 0.0)), 0.0)

        controller = self.params("controller_server").get("PidController", {})
        self.assertTrue(CONTROLLER_KEYS.issubset(controller))
        for key in CONTROLLER_KEYS:
            with self.subTest(key=key):
                self.assertIsNotNone(controller[key])
                self.assertGreaterEqual(float(controller[key]), 0.0)

        self.assertLessEqual(float(controller["max_speed"]), 2.0)
        self.assertLessEqual(float(controller["max_acceleration"]), 2.0)


if __name__ == "__main__":
    unittest.main()
