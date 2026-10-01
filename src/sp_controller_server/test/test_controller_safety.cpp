#include <chrono>

#include <gtest/gtest.h>

#include "sp_controller_server/controller_safety.hpp"

namespace {

using namespace std::chrono_literals;
using sp_controller_server::OdometryFreshnessGate;

TEST(OdometryFreshnessGate, RejectsMissingAndStaleSamples)
{
  OdometryFreshnessGate gate;
  const auto t0 = OdometryFreshnessGate::Clock::time_point{};

  EXPECT_FALSE(gate.isFresh(t0, 300ms));
  gate.observe(t0);
  EXPECT_TRUE(gate.isFresh(t0 + 299ms, 300ms));
  EXPECT_FALSE(gate.isFresh(t0 + 301ms, 300ms));
}

TEST(OdometryFreshnessGate, RejectsInvalidTimeoutAndBackwardTime)
{
  OdometryFreshnessGate gate;
  const auto t0 = OdometryFreshnessGate::Clock::time_point{} + 1s;
  gate.observe(t0);

  EXPECT_FALSE(gate.isFresh(t0, 0ms));
  EXPECT_FALSE(gate.isFresh(t0 - 1ms, 300ms));
}

}  // namespace
