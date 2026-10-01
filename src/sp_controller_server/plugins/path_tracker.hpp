#pragma once

#include <cstddef>
#include <vector>

namespace pid_controller {

struct Point2D
{
  double x;
  double y;
};

struct Velocity2D
{
  double x;
  double y;
};

struct TrackerParams
{
  double kp;
  double ki;
  double kd;
  double integral_limit;
  double lookahead_distance;
  double max_speed;
  double max_acceleration;
  double corner_slowdown_angle;
  double corner_speed_ratio;
  double goal_slowdown_distance;
  double goal_tolerance;
};

struct TrackerOutput
{
  Velocity2D command;
  std::size_t nearest_index;
  bool goal_reached;
};

class PathTracker
{
public:
  explicit PathTracker(TrackerParams params);

  void setPath(std::vector<Point2D> path);
  TrackerOutput step(Point2D position, Velocity2D velocity, double dt);
  const std::vector<Point2D> & path() const;

private:
  TrackerParams params_;
  std::vector<Point2D> path_;
  std::size_t nearest_index_{0};
  Velocity2D integral_{0.0, 0.0};
  Velocity2D previous_command_{0.0, 0.0};
};

}  // namespace pid_controller
