#!/usr/bin/env python3

from pathlib import Path
import unittest

import yaml


CONFIG = Path(__file__).resolve().parents[1] / "src/sp_nav_bringup/rviz/rviz.rviz"


class RvizEvidenceConfigTest(unittest.TestCase):
    def setUp(self) -> None:
        self.document = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        self.manager = self.document["Visualization Manager"]
        self.displays = self.manager["Displays"]

    def display_by_topic(self, topic: str):
        for display in self.displays:
            if display.get("Topic", {}).get("Value") == topic:
                return display
        self.fail(f"RViz display for {topic} is missing")

    def test_required_evidence_displays_are_enabled(self) -> None:
        for topic in ("/global_costmap", "/global_path", "/plan_markers"):
            display = self.display_by_topic(topic)
            self.assertTrue(display.get("Enabled"), topic)
            self.assertTrue(display.get("Value"), topic)

        tf_display = next(d for d in self.displays if d.get("Class") == "rviz_default_plugins/TF")
        self.assertTrue(tf_display.get("Enabled"))

    def test_unused_high_load_displays_are_disabled(self) -> None:
        for topic in (
            "/local_costmap/costmap",
            "/dynamic_obstacles/markers",
            "/filtered_cloud",
            "/MincoLocalPlanner/waypoints",
            "/MincoLocalPlanner/init_waypoints",
            "/controller_server/minco_mpc_cmd_vel_arrow",
            "/controller_server/minco_mpc_reference_traj",
        ):
            display = self.display_by_topic(topic)
            self.assertFalse(display.get("Enabled"), topic)
            self.assertFalse(display.get("Value"), topic)

    def test_frame_rate_is_bounded_for_software_rendering(self) -> None:
        self.assertLessEqual(self.manager["Global Options"]["Frame Rate"], 10)


if __name__ == "__main__":
    unittest.main()
