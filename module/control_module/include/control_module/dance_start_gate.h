#pragma once

#include <chrono>
#include <cmath>
#include <stdexcept>

namespace xyber_x1_infer::rl_control_module {

// Only qualifies readiness; it never creates or queues a start event.
class DanceStartGate {
 public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  void Configure(double hold_s) {
    if (!std::isfinite(hold_s) || hold_s < 0 || hold_s > 60) {
      throw std::runtime_error("dance_ready_hold_s must be in [0, 60]");
    }
    hold_ = std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(hold_s));
    Reset();
  }

  void Reset() { tracking_ = false; }

  void Update(bool ready, TimePoint now = Clock::now()) {
    if (!ready) {
      Reset();
      return;
    }
    // A scheduling/feedback gap cannot count as continuous readiness.
    if (!tracking_ || now < last_ || now - last_ > kMaxGap) since_ = now;
    tracking_ = true;
    last_ = now;
  }

  bool Ready(TimePoint now = Clock::now()) const {
    return tracking_ && now >= last_ && now - last_ <= kMaxGap && now - since_ >= hold_;
  }

 private:
  static constexpr auto kMaxGap = std::chrono::milliseconds(80);
  Clock::duration hold_{std::chrono::seconds(4)};
  TimePoint since_{}, last_{};
  bool tracking_{false};
};

}  // namespace xyber_x1_infer::rl_control_module
