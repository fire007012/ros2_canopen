// Copyright 2026 ros2_canopen contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef CANOPEN_402_DRIVER_COMMAND_SESSION_HPP_
#define CANOPEN_402_DRIVER_COMMAND_SESSION_HPP_

#include <cstdint>

namespace ros2_canopen
{
// Motor402 serializes this state with mode_mutex_. A revoked epoch never becomes valid again.
class CommandSession
{
public:
  void revoke() { ready_ = false; ++epoch_; }
  void confirm() { ready_ = true; }
  bool ready() const { return ready_; }
  uint64_t epoch() const { return epoch_; }
  bool accepts(uint64_t epoch) const { return ready_ && epoch == epoch_; }

private:
  uint64_t epoch_ = 0;
  bool ready_ = false;
};
}  // namespace ros2_canopen
#endif
