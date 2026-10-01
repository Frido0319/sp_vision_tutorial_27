#!/usr/bin/env python3

import argparse
import json
import math
import os
from pathlib import Path
import statistics
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_msg.msg import RobotKinematicsArray

from tools.nav_metrics import goal_distance, grid_cost, point_to_path_distance


START = (0.900, 0.900)
GOAL = (14.100, 14.100)
START_TOLERANCE_M = 0.15
SUCCESS_TOLERANCE_M = 0.30
SUCCESS_SAMPLES = 5


class NavigationEvaluator(Node):
    def __init__(self, timeout: float, startup_timeout: float, output: Path, sim_pub_hz: float):
        super().__init__("navigation_evaluator")
        self.timeout = timeout
        self.startup_timeout = startup_timeout
        self.output = output
        self.sim_pub_hz = sim_pub_hz
        self.started_at = time.monotonic()
        self.goal_started_at = None
        self.goal_publish_count = 0
        self.finished = False
        self.exit_code = 2
        self.reason = "startup_incomplete"

        self.odom = None
        self.initial_world = None
        self.odom_origin_world = None
        self.position = None
        self.previous_position = None
        self.previous_position_time = None
        self.path = []
        self.global_costmap = None
        self.latest_command = (0.0, 0.0)
        self.non_finite_command = False
        self.consecutive_goal_samples = 0
        self.stationary_started_at = None
        self.max_stationary_commanded = 0.0

        self.cross_track_samples = []
        self.command_samples = []
        self.odom_samples = 0
        self.odom_intervals = []
        self.path_update_count = 0
        self.max_observed_cost = None
        self.lethal_cost_samples = 0

        self.create_subscription(Odometry, "/Odometry", self._on_odom, 10)
        self.create_subscription(NavPath, "/global_path", self._on_path, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap", self._on_costmap, 10)
        self.create_subscription(Twist, "/sentry/cmd_vel", self._on_command, 10)
        self.create_subscription(RobotKinematicsArray, "/robots", self._on_robots, 10)
        goal_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.goal_publisher = self.create_publisher(PoseStamped, "/goal_pose", goal_qos)

    def _on_robots(self, message):
        if self.initial_world is not None:
            return
        for robot in message.robots:
            if robot.id == 1:
                self.initial_world = (
                    float(robot.pose.pose.position.x),
                    float(robot.pose.pose.position.y),
                )
                self._establish_origin()
                return

    def _on_odom(self, message):
        self.odom = message
        self._establish_origin()
        if self.odom_origin_world is None:
            return

        now = time.monotonic()
        if self.previous_position_time is not None:
            self.odom_intervals.append(now - self.previous_position_time)
        self.position = (
            self.odom_origin_world[0] + float(message.pose.pose.position.x),
            self.odom_origin_world[1] + float(message.pose.pose.position.y),
        )
        self.odom_samples += 1
        if self.goal_started_at is None:
            self.previous_position = self.position
            self.previous_position_time = now
            return

        if self.path:
            error = point_to_path_distance(self.position, self.path)
            if math.isfinite(error):
                self.cross_track_samples.append(error)

        if self.global_costmap is not None:
            info = self.global_costmap.info
            cost = grid_cost(
                self.position,
                (float(info.origin.position.x), float(info.origin.position.y)),
                float(info.resolution),
                int(info.width),
                int(info.height),
                self.global_costmap.data,
            )
            if cost is not None:
                self.max_observed_cost = (
                    cost if self.max_observed_cost is None else max(self.max_observed_cost, cost)
                )
                if cost >= 90:
                    self.lethal_cost_samples += 1
                    self.finish(8, f"robot_in_lethal_cost:{cost}")

        self._track_stationary_interval(now)
        final_error = goal_distance(self.position, GOAL)
        if final_error <= SUCCESS_TOLERANCE_M:
            self.consecutive_goal_samples += 1
        else:
            self.consecutive_goal_samples = 0

        if self.consecutive_goal_samples >= SUCCESS_SAMPLES:
            self.finish(0, "arrived")

        self.previous_position = self.position
        self.previous_position_time = now

    def _establish_origin(self):
        if self.odom_origin_world is not None or self.odom is None or self.initial_world is None:
            return
        self.odom_origin_world = (
            self.initial_world[0] - float(self.odom.pose.pose.position.x),
            self.initial_world[1] - float(self.odom.pose.pose.position.y),
        )

    def _on_path(self, message):
        self.path_update_count += 1
        self.path = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses
            if math.isfinite(pose.pose.position.x) and math.isfinite(pose.pose.position.y)
        ]

    def _on_costmap(self, message):
        self.global_costmap = message

    def _on_command(self, message):
        command = (float(message.linear.x), float(message.linear.y))
        if not all(math.isfinite(value) for value in command):
            self.non_finite_command = True
            self.latest_command = (0.0, 0.0)
            self.finish(3, "non_finite_command")
            return
        self.latest_command = command
        self.command_samples.append(math.hypot(*command))

    def _track_stationary_interval(self, now):
        command_speed = math.hypot(*self.latest_command)
        measured_speed = math.inf
        if self.previous_position is not None and self.previous_position_time is not None:
            dt = now - self.previous_position_time
            if dt > 0.0:
                measured_speed = goal_distance(self.position, self.previous_position) / dt

        if command_speed >= 0.20 and measured_speed <= 0.02:
            if self.stationary_started_at is None:
                self.stationary_started_at = now
            duration = now - self.stationary_started_at
            self.max_stationary_commanded = max(self.max_stationary_commanded, duration)
            if duration >= 1.50:
                self.finish(4, "commanded_but_stationary")
        else:
            self.stationary_started_at = None

    def ready(self):
        return (
            self.odom_origin_world is not None
            and self.count_publishers("/global_path") > 0
            and self.count_subscribers("/goal_pose") >= 2
            and self.count_subscribers("/sentry/cmd_vel") > 0
        )

    def publish_goal_once(self):
        if self.goal_publish_count != 0:
            raise RuntimeError("evaluator attempted to publish more than one goal")
        start_error = goal_distance(self.initial_world, START)
        if start_error > START_TOLERANCE_M:
            self.finish(5, f"invalid_start:{start_error:.3f}m")
            return

        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "map"
        message.pose.position.x = GOAL[0]
        message.pose.position.y = GOAL[1]
        message.pose.orientation.w = 1.0
        self.goal_publisher.publish(message)
        self.goal_publish_count = 1
        self.goal_started_at = time.monotonic()
        self.get_logger().info(
            f"Published the only goal: ({GOAL[0]:.3f}, {GOAL[1]:.3f})"
        )

    def finish(self, code, reason):
        if self.finished:
            return
        self.exit_code = code
        self.reason = reason
        self.finished = True

    def check_deadlines(self):
        now = time.monotonic()
        if self.goal_started_at is None:
            if now - self.started_at >= self.startup_timeout:
                self.finish(6, "startup_timeout")
        elif now - self.goal_started_at >= self.timeout:
            self.finish(7, "navigation_timeout")

    def write_result(self):
        elapsed = None if self.goal_started_at is None else time.monotonic() - self.goal_started_at
        final_error = None if self.position is None else goal_distance(self.position, GOAL)
        result = {
            "success": self.exit_code == 0,
            "status": self.reason,
            "start": list(START),
            "measured_start": None if self.initial_world is None else list(self.initial_world),
            "goal": list(GOAL),
            "goal_publish_count": self.goal_publish_count,
            "local_evaluation_sim_pub_hz": self.sim_pub_hz,
            "elapsed_seconds": elapsed,
            "final_error_m": final_error,
            "mean_cross_track_error_m": (
                statistics.fmean(self.cross_track_samples) if self.cross_track_samples else None
            ),
            "max_cross_track_error_m": (
                max(self.cross_track_samples) if self.cross_track_samples else None
            ),
            "mean_command_speed_mps": (
                statistics.fmean(self.command_samples) if self.command_samples else None
            ),
            "max_command_speed_mps": max(self.command_samples) if self.command_samples else None,
            "max_commanded_but_stationary_seconds": self.max_stationary_commanded,
            "non_finite_command": self.non_finite_command,
            "odom_samples": self.odom_samples,
            "mean_odom_interval_seconds": (
                statistics.fmean(self.odom_intervals) if self.odom_intervals else None
            ),
            "max_odom_interval_seconds": max(self.odom_intervals) if self.odom_intervals else None,
            "path_points": len(self.path),
            "path_update_count": self.path_update_count,
            "max_observed_global_cost": self.max_observed_cost,
            "lethal_cost_samples": self.lethal_cost_samples,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output.with_suffix(self.output.suffix + ".tmp")
        temporary.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        os.replace(temporary, self.output)
        print(json.dumps(result, indent=2, allow_nan=False))


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate one fixed maze-navigation run")
    parser.add_argument("--timeout", type=float, default=180.0)
    parser.add_argument("--startup-timeout", type=float, default=45.0)
    parser.add_argument("--output", type=Path, default=Path("artifacts/nav-evaluation.json"))
    parser.add_argument("--sim-pub-hz", type=float, default=30.0)
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    evaluator = NavigationEvaluator(
        args.timeout, args.startup_timeout, args.output, args.sim_pub_hz
    )
    ready_since = None
    try:
        while rclpy.ok() and not evaluator.finished:
            rclpy.spin_once(evaluator, timeout_sec=0.1)
            if evaluator.goal_started_at is None:
                if evaluator.ready():
                    if ready_since is None:
                        ready_since = time.monotonic()
                    elif time.monotonic() - ready_since >= 2.0:
                        evaluator.publish_goal_once()
                else:
                    ready_since = None
            evaluator.check_deadlines()
        evaluator.write_result()
        return evaluator.exit_code
    finally:
        evaluator.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
