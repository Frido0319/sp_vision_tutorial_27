#!/usr/bin/env python3

from pathlib import Path
import unittest


SCRIPT = (
    Path(__file__).resolve().parents[1] / "bash/run_nav_gui.sh"
).read_text(encoding="utf-8")


class GuiLauncherContractTest(unittest.TestCase):
    def test_simulator_uses_headless_pygame_while_rviz_remains_visible(self) -> None:
        self.assertIn(
            "start_group gui-simulator.log env SDL_VIDEODRIVER=dummy",
            SCRIPT,
            "the pygame simulator window must not starve ROS timers during RViz evidence runs",
        )
        self.assertIn(
            "ros2 launch sp_nav_bringup sp_nav.launch.py use_rviz:=true",
            SCRIPT,
        )
        self.assertNotIn("export SDL_VIDEODRIVER=dummy", SCRIPT)


if __name__ == "__main__":
    unittest.main()
