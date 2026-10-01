#!/usr/bin/env python3

"""Static safety contract for the one-click navigation behavior tree."""

from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


REPO = Path(__file__).resolve().parents[1]
TREE_FILE = REPO / "src/sp_decision/decision_trees/click_nav.xml"
ENGINE_FILE = REPO / "src/sp_decision/src/decision_engine.cpp"


class ClickNavigationContractTest(unittest.TestCase):
    def setUp(self) -> None:
        self.root = ET.parse(TREE_FILE).getroot()

    def test_nav_failure_is_not_hidden_by_force_success(self) -> None:
        """A failed/aborted NavigateToPose action must reach the engine as FAILURE."""
        parent = {
            child: node for node in self.root.iter() for child in list(node)
        }
        nav_nodes = list(self.root.iter("NavToPose"))
        self.assertEqual(len(nav_nodes), 1, "click_nav.xml must contain one NavToPose")

        node = nav_nodes[0]
        ancestors = []
        while node in parent:
            node = parent[node]
            ancestors.append(node.tag)

        self.assertNotIn(
            "ForceSuccess",
            ancestors,
            "NavToPose is wrapped in ForceSuccess, so ABORTED is logged as SUCCEEDED",
        )

    def test_goal_is_consumed_once_before_navigation(self) -> None:
        set_nodes = [
            node
            for node in self.root.iter("SetDouble")
            if node.attrib.get("key") == "clicked_point_update"
        ]
        self.assertEqual(len(set_nodes), 1)
        self.assertEqual(set_nodes[0].attrib.get("value"), "0.0")

    def test_tree_cycle_log_does_not_claim_navigation_success(self) -> None:
        source = ENGINE_FILE.read_text(encoding="utf-8")
        self.assertNotIn(
            'notify("SUCCEEDED – restarting")',
            source,
            "the setup branch also returns SUCCESS, so this message is not arrival evidence",
        )


if __name__ == "__main__":
    unittest.main()
