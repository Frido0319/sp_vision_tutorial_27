#include "pid_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>

#include <rclcpp/exceptions.hpp>
#include <sstream>
#include <stdexcept>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace {

[[noreturn]] void sp_nav_param_error(const rclcpp::Node & node, const std::string & name)
{
  std::ostringstream oss;
  oss << "[参数缺失] 节点 '" << node.get_name() << "' 缺少参数 '" << name
      << "'，请在对应 yaml 的 ros__parameters 中填写。";
  throw std::runtime_error(oss.str());
}

template<typename T>
T require_param(const rclcpp::Node::SharedPtr & node, const std::string & name)
{
  try {
    if (node->has_parameter(name)) {
      return node->get_parameter(name).get_value<T>();
    }
    return node->declare_parameter<T>(name);
  } catch (const rclcpp::exceptions::UninitializedStaticallyTypedParameterException &) {
    sp_nav_param_error(*node, name);
  }
}

void validate_params(const pid_controller::TrackerParams & params, const double frequency)
{
  const auto finite = [](const double value) {return std::isfinite(value);};
  if (!finite(frequency) || frequency <= 0.0) {
    throw std::invalid_argument("control_frequency must be finite and positive");
  }
  if (!finite(params.kp) || params.kp < 0.0 ||
    !finite(params.ki) || params.ki < 0.0 ||
    !finite(params.kd) || params.kd < 0.0)
  {
    throw std::invalid_argument("PID gains must be finite and non-negative");
  }
  if (!finite(params.integral_limit) || params.integral_limit < 0.0 ||
    !finite(params.lookahead_distance) || params.lookahead_distance <= 0.0 ||
    !finite(params.max_speed) || params.max_speed <= 0.0 ||
    !finite(params.max_acceleration) || params.max_acceleration <= 0.0 ||
    !finite(params.corner_slowdown_angle) || params.corner_slowdown_angle < 0.0 ||
    !finite(params.corner_speed_ratio) || params.corner_speed_ratio <= 0.0 ||
    params.corner_speed_ratio > 1.0 ||
    !finite(params.goal_slowdown_distance) || params.goal_slowdown_distance <= 0.0 ||
    !finite(params.goal_tolerance) || params.goal_tolerance <= 0.0)
  {
    throw std::invalid_argument("tracker limits and tolerances are invalid");
  }
}

}

namespace pid_controller {

void PidController::configure(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const std::shared_ptr<tf2_ros::Buffer> & tf_buffer)
{
  node_ = node;
  plugin_name_ = name;
  tf_buffer_ = tf_buffer;
  if (!node_) {
    throw std::runtime_error("PidController received null node");
  }
  base_frame_id_ = require_param<std::string>(node_, plugin_name_ + ".base_frame_id");
  control_frequency_ = require_param<double>(node_, "control_frequency");
  const TrackerParams params{
    require_param<double>(node_, plugin_name_ + ".kp"),
    require_param<double>(node_, plugin_name_ + ".ki"),
    require_param<double>(node_, plugin_name_ + ".kd"),
    require_param<double>(node_, plugin_name_ + ".integral_limit"),
    require_param<double>(node_, plugin_name_ + ".lookahead_distance"),
    require_param<double>(node_, plugin_name_ + ".max_speed"),
    require_param<double>(node_, plugin_name_ + ".max_acceleration"),
    require_param<double>(node_, plugin_name_ + ".corner_slowdown_angle"),
    require_param<double>(node_, plugin_name_ + ".corner_speed_ratio"),
    require_param<double>(node_, plugin_name_ + ".goal_slowdown_distance"),
    require_param<double>(node_, plugin_name_ + ".goal_tolerance")};
  validate_params(params, control_frequency_);
  tracker_ = std::make_unique<PathTracker>(params);
  last_step_time_ = std::chrono::steady_clock::now();
  has_last_step_time_ = false;
}

void PidController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  if (!tracker_) {
    throw std::runtime_error("PidController must be configured before setPlan");
  }
  std::vector<Point2D> points;
  points.reserve(path.poses.size());
  for (const auto & pose : path.poses) {
    points.push_back({pose.pose.position.x, pose.pose.position.y});
  }
  tracker_->setPath(std::move(points));
  has_last_step_time_ = false;
}

geometry_msgs::msg::TwistStamped PidController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity)
{
  if (!tracker_) {
    throw std::runtime_error("PidController is not configured");
  }

  const auto now = std::chrono::steady_clock::now();
  double dt = 1.0 / control_frequency_;
  if (has_last_step_time_) {
    dt = std::chrono::duration<double>(now - last_step_time_).count();
    dt = std::clamp(dt, 1e-4, 0.1);
  }
  last_step_time_ = now;
  has_last_step_time_ = true;

  const TrackerOutput output = tracker_->step(
    {pose.pose.position.x, pose.pose.position.y},
    {velocity.linear.x, velocity.linear.y}, dt);

  tf2::Quaternion orientation;
  tf2::fromMsg(pose.pose.orientation, orientation);
  const double yaw = tf2::getYaw(orientation);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.stamp = node_->now();
  cmd_vel.header.frame_id = base_frame_id_;
  cmd_vel.twist.linear.x = c * output.command.x + s * output.command.y;
  cmd_vel.twist.linear.y = -s * output.command.x + c * output.command.y;
  cmd_vel.twist.angular.z = 0.0;
  return cmd_vel;
}

}

PLUGINLIB_EXPORT_CLASS(pid_controller::PidController, sp_controller_server::ControllerPlugin)
