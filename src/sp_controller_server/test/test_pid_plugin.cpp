#include <cmath>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <gtest/gtest.h>
#include <nav_msgs/msg/path.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>

#include "sp_controller_server/controller_plugin.hpp"

namespace {

class RclcppEnvironment : public testing::Environment
{
public:
  void SetUp() override
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  void TearDown() override
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

testing::Environment * const environment =
  testing::AddGlobalTestEnvironment(new RclcppEnvironment());

TEST(PidControllerPlugin, LoadsConfiguresAndTracksPath)
{
  (void)environment;
  auto node = std::make_shared<rclcpp::Node>("pid_plugin_test");
  node->declare_parameter("control_frequency", 50.0);
  node->declare_parameter("PidController.base_frame_id", "base_link");
  node->declare_parameter("PidController.kp", 1.4);
  node->declare_parameter("PidController.ki", 0.02);
  node->declare_parameter("PidController.kd", 0.35);
  node->declare_parameter("PidController.integral_limit", 0.3);
  node->declare_parameter("PidController.lookahead_distance", 0.45);
  node->declare_parameter("PidController.max_speed", 1.0);
  node->declare_parameter("PidController.max_acceleration", 1.5);
  node->declare_parameter("PidController.corner_slowdown_angle", 0.65);
  node->declare_parameter("PidController.corner_speed_ratio", 0.45);
  node->declare_parameter("PidController.goal_slowdown_distance", 1.0);
  node->declare_parameter("PidController.goal_tolerance", 0.15);

  pluginlib::ClassLoader<sp_controller_server::ControllerPlugin> loader(
    "sp_controller_server", "sp_controller_server::ControllerPlugin");
  auto controller = loader.createSharedInstance("PidController");
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  controller->configure(node, "PidController", tf_buffer);

  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped first;
  first.header.frame_id = "map";
  first.pose.orientation.w = 1.0;
  first.pose.position.x = 0.0;
  first.pose.position.y = 0.0;
  geometry_msgs::msg::PoseStamped second = first;
  second.pose.position.x = 2.0;
  path.poses = {first, second};
  controller->setPlan(path);

  geometry_msgs::msg::PoseStamped pose = first;
  geometry_msgs::msg::Twist velocity;
  const auto command = controller->computeVelocityCommands(pose, velocity);

  EXPECT_EQ(command.header.frame_id, "base_link");
  EXPECT_TRUE(std::isfinite(command.twist.linear.x));
  EXPECT_TRUE(std::isfinite(command.twist.linear.y));
  EXPECT_GT(command.twist.linear.x, 0.0);
  EXPECT_NEAR(command.twist.linear.y, 0.0, 1e-9);

  controller->setPlan(path);
  pose.pose.orientation.z = std::sqrt(0.5);
  pose.pose.orientation.w = std::sqrt(0.5);
  const auto rotated_command = controller->computeVelocityCommands(pose, velocity);
  EXPECT_NEAR(rotated_command.twist.linear.x, 0.0, 1e-9);
  EXPECT_LT(rotated_command.twist.linear.y, 0.0);
}

}  // namespace
