// SPDX-License-Identifier: Apache-2.0
#ifndef CANOPEN_MOTION_HEALTH_HPP_
#define CANOPEN_MOTION_HEALTH_HPP_
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <map>
#include <stdexcept>
namespace ros2_canopen {
// Every required PDO channel has its own warning, stopping and recovery deadlines.
class MotionHealth {
public:
  using Clock = std::chrono::steady_clock;
  struct Snapshot { bool ready; uint64_t epoch; std::string reason; bool warning = false; uint16_t emcy = 0; };
  void configure(bool enabled, std::chrono::milliseconds timeout) {
    if (timeout.count() < 2) throw std::invalid_argument("feedback deadline must exceed 1 ms");
    std::lock_guard<std::mutex> lock(mutex_); enabled_ = enabled;
    for (auto index : {0x6041,0x6061,0x6064,0x606C}) {
      auto & c = channels_[index]; c.stop = timeout; c.warn = timeout/2; c.stable = std::chrono::milliseconds(0);
    }
  }
  void timing(uint16_t index, std::chrono::milliseconds warn, std::chrono::milliseconds stop,
              std::chrono::milliseconds stable) {
    if (warn.count() <= 0 || stop <= warn || stable.count() < 0)
      throw std::invalid_argument("require 0 < warn < stop and stable >= 0");
    std::lock_guard<std::mutex> lock(mutex_);
    auto & c = channels_.at(index); c.warn=warn; c.stop=stop; c.stable=stable;
    c.established=false; c.since={};
  }
  void nmt(bool operational) {
    std::lock_guard<std::mutex> lock(mutex_); operational_=operational;
    if (!operational) trip("NMT not operational");
  }
  void heartbeat(bool occurred) {
    std::lock_guard<std::mutex> lock(mutex_); heartbeat_failed_=occurred;
    if (occurred) trip("heartbeat timeout");
  }
  void emergency(uint16_t code) {
    std::lock_guard<std::mutex> lock(mutex_); emergency_=code;
    if (code) trip("EMCY " + std::to_string(code));
  }
  void feedback(uint16_t index, Clock::time_point now=Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_); expire(now);
    auto it=channels_.find(index); if(it!=channels_.end())it->second.stamp=now;
  }
  Snapshot snapshot(Clock::time_point now=Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled_) return {true,epoch_,"disabled",false,emergency_};
    expire(now); bool fresh=true, warning=false;
    for (auto & entry:channels_) {
      auto & c=entry.second;
      if(c.stamp==Clock::time_point{} || now<c.stamp) {fresh=false;continue;}
      const auto age=now-c.stamp;
      if(age>=c.warn) {c.warned=true;c.since={};}
      else {
        if(c.since==Clock::time_point{})c.since=now;
        if(now-c.since>=c.stable) {c.established=true;c.warned=false;}
      }
      fresh=fresh&&c.established&&age<c.stop; warning=warning||c.warned;
    }
    const bool ready=operational_&&!heartbeat_failed_&&!emergency_&&fresh;
    return {ready,epoch_,ready?(warning?"PDO feedback warning":"ready"):reason_,warning,emergency_};
  }
private:
  struct Channel {
    Clock::time_point stamp{},since{};
    std::chrono::milliseconds warn{50},stop{100},stable{0};
    bool established=false,warned=false;
  };
  void trip(const std::string & reason) {
    ++epoch_;reason_=reason;
    for(auto & entry:channels_) {entry.second.established=false;entry.second.since={};}
  }
  void expire(Clock::time_point now) {
    if(!enabled_)return;
    for(auto & entry:channels_) {
      auto & c=entry.second;
      if(c.stamp!=Clock::time_point{}&&(now<c.stamp||now-c.stamp>=c.stop)) {
        c.stamp={};c.warned=false;trip("PDO feedback stale");
      }
    }
  }
  std::mutex mutex_;
  bool enabled_=false,operational_=false,heartbeat_failed_=false;
  uint16_t emergency_=0;
  uint64_t epoch_=0;
  std::map<uint16_t,Channel> channels_{{0x6041,{}},{0x6061,{}},{0x6064,{}},{0x606C,{}}};
  std::string reason_="waiting for stable PDO feedback";
};
}
#endif
