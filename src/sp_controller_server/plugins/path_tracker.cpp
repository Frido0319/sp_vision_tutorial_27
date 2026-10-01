#include "path_tracker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace pid_controller {
namespace {

constexpr double kEpsilon = 1e-9;

bool finite(const Point2D point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool finite(const Velocity2D velocity)
{
  return std::isfinite(velocity.x) && std::isfinite(velocity.y);
}

double distance(const Point2D a, const Point2D b)
{
  return std::hypot(a.x - b.x, a.y - b.y);
}

double norm(const Velocity2D velocity)
{
  return std::hypot(velocity.x, velocity.y);
}

Velocity2D limitNorm(Velocity2D value, const double limit)
{
  const double magnitude = norm(value);
  if (!std::isfinite(magnitude) || limit <= 0.0) {
    return {0.0, 0.0};
  }
  if (magnitude > limit) {
    value.x *= limit / magnitude;
    value.y *= limit / magnitude;
  }
  return value;
}

double positiveOr(const double value, const double fallback)
{
  return std::isfinite(value) && value > 0.0 ? value : fallback;
}

double nonNegativeOr(const double value, const double fallback)
{
  return std::isfinite(value) && value >= 0.0 ? value : fallback;
}

}  // namespace

PathTracker::PathTracker(TrackerParams params)
: params_(std::move(params))
{
  params_.kp = nonNegativeOr(params_.kp, 0.0);
  params_.ki = nonNegativeOr(params_.ki, 0.0);
  params_.kd = nonNegativeOr(params_.kd, 0.0);
  params_.integral_limit = nonNegativeOr(params_.integral_limit, 0.0);
  params_.lookahead_distance = positiveOr(params_.lookahead_distance, 0.1);
  params_.max_speed = positiveOr(params_.max_speed, 0.1);
  params_.max_acceleration = positiveOr(params_.max_acceleration, 0.1);
  params_.corner_slowdown_angle = nonNegativeOr(params_.corner_slowdown_angle, 0.0);
  params_.corner_speed_ratio = std::clamp(
    nonNegativeOr(params_.corner_speed_ratio, 1.0), 0.0, 1.0);
  params_.goal_slowdown_distance = positiveOr(params_.goal_slowdown_distance, 0.1);
  params_.goal_tolerance = nonNegativeOr(params_.goal_tolerance, 0.0);
}

void PathTracker::setPath(std::vector<Point2D> path)
{
  path_.clear();
  path_.reserve(path.size());
  for (const Point2D point : path) {
    if (!finite(point)) {
      continue;
    }
    if (path_.empty() || distance(path_.back(), point) > kEpsilon) {
      path_.push_back(point);
    }
  }

  nearest_index_ = 0;
  integral_ = {0.0, 0.0};
  if (path_.empty()) {
    previous_command_ = {0.0, 0.0};
  }
}

TrackerOutput PathTracker::step(
  const Point2D position, const Velocity2D velocity, const double dt)
{
  const auto fail_safe = [this]() {
      previous_command_ = {0.0, 0.0};
      return TrackerOutput{{0.0, 0.0}, nearest_index_, false};
    };

  if (path_.empty() || !finite(position) || !finite(velocity) ||
    !std::isfinite(dt) || dt <= 0.0)
  {
    return fail_safe();
  }

  nearest_index_ = std::min(nearest_index_, path_.size() - 1);
  double nearest_distance = distance(position, path_[nearest_index_]);
  while (nearest_index_ + 1 < path_.size()) {
    const double next_distance = distance(position, path_[nearest_index_ + 1]);
    if (next_distance > nearest_distance + kEpsilon) {
      break;
    }
    ++nearest_index_;
    nearest_distance = next_distance;
  }

  const double distance_to_goal = distance(position, path_.back());
  if (distance_to_goal <= params_.goal_tolerance) {
    integral_ = {0.0, 0.0};
    previous_command_ = {0.0, 0.0};
    return {{0.0, 0.0}, nearest_index_, true};
  }

  Point2D target = path_[nearest_index_];
  std::size_t target_index = nearest_index_;
  double remaining = params_.lookahead_distance;
  const auto corner_angle = [this](const std::size_t index) {
      if (index == 0 || index + 1 >= path_.size()) {
        return 0.0;
      }
      const Velocity2D incoming{
        path_[index].x - path_[index - 1].x,
        path_[index].y - path_[index - 1].y};
      const Velocity2D outgoing{
        path_[index + 1].x - path_[index].x,
        path_[index + 1].y - path_[index].y};
      const double denominator = norm(incoming) * norm(outgoing);
      if (denominator <= kEpsilon) {
        return 0.0;
      }
      const double cosine = std::clamp(
        (incoming.x * outgoing.x + incoming.y * outgoing.y) / denominator,
        -1.0, 1.0);
      return std::acos(cosine);
    };
  for (std::size_t index = nearest_index_; index + 1 < path_.size(); ++index) {
    const double segment = distance(path_[index], path_[index + 1]);
    if (segment <= kEpsilon) {
      continue;
    }
    if (segment >= remaining) {
      const double ratio = remaining / segment;
      target = {
        path_[index].x + ratio * (path_[index + 1].x - path_[index].x),
        path_[index].y + ratio * (path_[index + 1].y - path_[index].y)};
      target_index = index + 1;
      remaining = 0.0;
      break;
    }
    const std::size_t next_index = index + 1;
    if (corner_angle(next_index) >= params_.corner_slowdown_angle &&
      corner_angle(next_index) > kEpsilon)
    {
      target = path_[next_index];
      target_index = next_index;
      remaining = 0.0;
      break;
    }
    remaining -= segment;
    target = path_[index + 1];
    target_index = index + 1;
  }

  double speed_limit = params_.max_speed;
  for (std::size_t index = nearest_index_ + 1;
    index <= target_index && index + 1 < path_.size(); ++index)
  {
    if (corner_angle(index) >= params_.corner_slowdown_angle) {
      speed_limit = std::min(speed_limit, params_.max_speed * params_.corner_speed_ratio);
    }
  }
  if (distance_to_goal < params_.goal_slowdown_distance) {
    speed_limit = std::min(
      speed_limit,
      params_.max_speed * distance_to_goal / params_.goal_slowdown_distance);
  }

  const Velocity2D position_error{target.x - position.x, target.y - position.y};
  const double target_distance = norm(position_error);
  const Velocity2D desired_velocity = target_distance > kEpsilon ?
    Velocity2D{
      speed_limit * position_error.x / target_distance,
      speed_limit * position_error.y / target_distance} :
    Velocity2D{0.0, 0.0};

  integral_.x = std::clamp(
    integral_.x + position_error.x * dt,
    -params_.integral_limit, params_.integral_limit);
  integral_.y = std::clamp(
    integral_.y + position_error.y * dt,
    -params_.integral_limit, params_.integral_limit);

  Velocity2D command{
    desired_velocity.x + params_.kp * position_error.x + params_.ki * integral_.x +
      params_.kd * (desired_velocity.x - velocity.x),
    desired_velocity.y + params_.kp * position_error.y + params_.ki * integral_.y +
      params_.kd * (desired_velocity.y - velocity.y)};
  command = limitNorm(command, speed_limit);

  const Velocity2D delta{
    command.x - previous_command_.x,
    command.y - previous_command_.y};
  const Velocity2D limited_delta = limitNorm(delta, params_.max_acceleration * dt);
  command = {
    previous_command_.x + limited_delta.x,
    previous_command_.y + limited_delta.y};
  if (!finite(command)) {
    return fail_safe();
  }

  previous_command_ = command;
  return {command, nearest_index_, false};
}

const std::vector<Point2D> & PathTracker::path() const
{
  return path_;
}

}  // namespace pid_controller
