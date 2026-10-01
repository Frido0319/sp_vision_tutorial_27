#!/usr/bin/env python3

import math
import unittest

from tools.nav_metrics import goal_distance, point_to_path_distance


class NavigationMetricsTest(unittest.TestCase):
    def test_goal_distance_is_euclidean(self):
        self.assertEqual(goal_distance((1.0, 2.0), (4.0, 6.0)), 5.0)

    def test_horizontal_segment(self):
        self.assertEqual(point_to_path_distance((2.0, 3.0), [(0.0, 0.0), (4.0, 0.0)]), 3.0)

    def test_vertical_segment(self):
        self.assertEqual(point_to_path_distance((3.0, 2.0), [(0.0, 0.0), (0.0, 4.0)]), 3.0)

    def test_diagonal_segment_and_endpoint_clamping(self):
        self.assertAlmostEqual(
            point_to_path_distance((1.0, 0.0), [(0.0, 0.0), (2.0, 2.0)]),
            math.sqrt(0.5),
        )
        self.assertEqual(
            point_to_path_distance((4.0, 2.0), [(0.0, 0.0), (2.0, 0.0)]),
            math.sqrt(8.0),
        )

    def test_empty_and_single_point_paths(self):
        self.assertTrue(math.isinf(point_to_path_distance((0.0, 0.0), [])))
        self.assertEqual(point_to_path_distance((4.0, 6.0), [(1.0, 2.0)]), 5.0)

    def test_non_finite_input_returns_infinity(self):
        self.assertTrue(
            math.isinf(point_to_path_distance((math.nan, 0.0), [(0.0, 0.0), (1.0, 0.0)]))
        )


if __name__ == "__main__":
    unittest.main()
