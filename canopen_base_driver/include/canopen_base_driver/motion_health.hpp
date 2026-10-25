// SPDX-License-Identifier: Apache-2.0
#ifndef CANOPEN_MOTION_HEALTH_HPP_
#define CANOPEN_MOTION_HEALTH_HPP_
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
namespace ros2_canopen
{
// Updated by Lely callbacks, queried by the motor thread. No ROS clock dependence.
class MotionHealth
{
public:
  using Clock = std::chrono::steady_clock;
  struct Snapshot { bool ready; uint64_t epoch; std::string reason; };
  void configure(bool enabled, std::chrono::milliseconds timeout)
  { std::lock_guard<std::mutex> lock(mutex_); enabled_ = enabled; timeout_ = timeout; }
  void nmt(bool operational)
  { std::lock_guard<std::mutex> lock(mutex_); operational_ = operational;
    if (!operational) trip("NMT not operational"); }
  void heartbeat(bool occurred)
  { std::lock_guard<std::mutex> lock(mutex_); heartbeat_failed_ = occurred;
    if (occurred) trip("heartbeat timeout"); }
  void emergency(uint16_t code)
  { std::lock_guard<std::mutex> lock(mutex_); emergency_ = code != 0;
    if (code) trip("EMCY " + std::to_string(code)); }
  void feedback(uint16_t index, Clock::time_point now = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Detect a gap even if communication recovered between hardware write cycles.
    expire(now);
    if (index == 0x6041) status_ = now;
    if (index == 0x6061) mode_ = now;
    if (index == 0x6064) position_ = now;
    if (index == 0x606C) velocity_ = now;
  }
  Snapshot snapshot(Clock::time_point now = Clock::now())
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) return {true, epoch_, "disabled"};
    expire(now);
    bool fresh = status_ != Clock::time_point{} && mode_ != Clock::time_point{} &&
      position_ != Clock::time_point{} && velocity_ != Clock::time_point{} &&
      now - status_ <= timeout_ && now - mode_ <= timeout_ &&
      now - position_ <= timeout_ && now - velocity_ <= timeout_;
    return {operational_ && !heartbeat_failed_ && !emergency_ && fresh, epoch_, reason_};
  }
private:
  void trip(const std::string & reason) { ++epoch_; reason_ = reason; }
  void expire(Clock::time_point now)
  {
    if (!enabled_) return;
    auto stale = [&](Clock::time_point & stamp) {
      if (stamp != Clock::time_point{} && now - stamp > timeout_)
      { stamp = {}; trip("PDO feedback stale"); }
    };
    stale(status_); stale(mode_); stale(position_); stale(velocity_);
  }
  std::mutex mutex_;
  bool enabled_ = false, operational_ = false, heartbeat_failed_ = false, emergency_ = false;
  uint64_t epoch_ = 0;
  std::chrono::milliseconds timeout_{100};
  Clock::time_point status_{}, mode_{}, position_{}, velocity_{};
  std::string reason_ = "waiting for PDO feedback";
};
}
#endif
