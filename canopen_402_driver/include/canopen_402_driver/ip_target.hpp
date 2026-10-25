// SPDX-License-Identifier: GPL-3.0-or-later
#ifndef CANOPEN_IP_TARGET_HPP_
#define CANOPEN_IP_TARGET_HPP_
#include <cstdint>
namespace ros2_canopen
{
template<class Write>
void write_ip_target(int32_t target, bool mirror, Write write)
{
  write(0x60C1, 1, target);
  if (mirror) write(0x607A, 0, target);
}
}
#endif
