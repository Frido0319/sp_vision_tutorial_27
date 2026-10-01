#pragma once

#include <chrono>

namespace sp_controller_server {

class OdometryFreshnessGate
{
public:
  using Clock = std::chrono::steady_clock;

  void observe(const Clock::time_point now)
  {
    last_update_ = now;
    has_sample_ = true;
  }

  bool isFresh(const Clock::time_point now, const Clock::duration timeout) const
  {
    if (!has_sample_ || timeout <= Clock::duration::zero() || now < last_update_) {
      return false;
    }
    return now - last_update_ <= timeout;
  }

private:
  Clock::time_point last_update_{};
  bool has_sample_{false};
};

}  // namespace sp_controller_server
