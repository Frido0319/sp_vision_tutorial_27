#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "path_tracker.hpp"

namespace {

pid_controller::TrackerParams params()
{
  return {
    1.4, 0.02, 0.35, 0.3, 0.45, 1.0, 1.5, 0.65, 0.45, 1.0, 0.15};
}

double norm(const pid_controller::Velocity2D & velocity)
{
  return std::hypot(velocity.x, velocity.y);
}

TEST(PathTracker, OwnsSanitizedPathAndResetsProgress)
{
  pid_controller::PathTracker tracker(params());
  tracker.setPath({{0.0, 0.0}, {0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  ASSERT_EQ(tracker.path().size(), 3U);

  auto progressed = tracker.step({1.2, 0.0}, {0.0, 0.0}, 0.02);
  EXPECT_GE(progressed.nearest_index, 1U);

  tracker.setPath({{10.0, 0.0}, {11.0, 0.0}});
  auto reset = tracker.step({10.0, 0.0}, {0.0, 0.0}, 0.02);
  EXPECT_EQ(reset.nearest_index, 0U);
}

TEST(PathTracker, ProgressNeverMovesBackwardWithNoisyPositions)
{
  pid_controller::PathTracker tracker(params());
  tracker.setPath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}, {3.0, 0.0}});

  std::size_t previous = 0;
  for (const double x : {0.1, 0.9, 1.2, 0.95, 1.8, 1.55, 2.7}) {
    const auto output = tracker.step({x, 0.02}, {0.0, 0.0}, 0.02);
    EXPECT_GE(output.nearest_index, previous);
    previous = output.nearest_index;
  }
}

TEST(PathTracker, LimitsSpeedAndAcceleration)
{
  pid_controller::PathTracker tracker(params());
  tracker.setPath({{0.0, 0.0}, {1.0, 0.0}, {3.0, 0.0}});

  const auto first = tracker.step({0.0, 0.0}, {0.0, 0.0}, 0.02);
  EXPECT_LE(norm(first.command), 1.0 + 1e-9);
  EXPECT_LE(norm(first.command), 1.5 * 0.02 + 1e-9);

  const auto second = tracker.step({0.0, 0.0}, first.command, 0.02);
  EXPECT_LE(norm(second.command) - norm(first.command), 1.5 * 0.02 + 1e-9);
  EXPECT_GT(second.command.x, 0.0);
  EXPECT_NEAR(second.command.y, 0.0, 1e-9);
}

TEST(PathTracker, SlowsForCornerAndStopsAtGoal)
{
  auto straight_params = params();
  straight_params.max_acceleration = 100.0;
  pid_controller::PathTracker straight(straight_params);
  straight.setPath({{0.0, 0.0}, {0.4, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  const auto straight_command = straight.step({0.0, 0.0}, {0.0, 0.0}, 0.02);

  pid_controller::PathTracker corner(straight_params);
  corner.setPath({{0.0, 0.0}, {0.4, 0.0}, {0.4, 1.0}, {0.4, 2.0}});
  const auto corner_command = corner.step({0.0, 0.0}, {0.0, 0.0}, 0.02);
  EXPECT_LT(norm(corner_command.command), norm(straight_command.command));

  const auto stopped = corner.step({0.4, 1.93}, {0.2, 0.0}, 0.02);
  EXPECT_TRUE(stopped.goal_reached);
  EXPECT_DOUBLE_EQ(stopped.command.x, 0.0);
  EXPECT_DOUBLE_EQ(stopped.command.y, 0.0);
}

TEST(PathTracker, InvalidInputsFailSafeToFiniteZero)
{
  pid_controller::PathTracker tracker(params());
  tracker.setPath({{0.0, 0.0}, {1.0, 0.0}});

  const auto invalid = tracker.step(
    {std::numeric_limits<double>::quiet_NaN(), 0.0}, {0.0, 0.0}, 0.02);
  EXPECT_TRUE(std::isfinite(invalid.command.x));
  EXPECT_TRUE(std::isfinite(invalid.command.y));
  EXPECT_DOUBLE_EQ(invalid.command.x, 0.0);
  EXPECT_DOUBLE_EQ(invalid.command.y, 0.0);
}

}  // namespace
