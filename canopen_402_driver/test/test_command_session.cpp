// Copyright 2026 ros2_canopen contributors
// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>
#include "canopen_402_driver/command_session.hpp"
#include "canopen_402_driver/motor.hpp"
#include "canopen_402_driver/mode_target_helper.hpp"

TEST(CommandEpoch, StartsClosedAndNeverRevalidatesOldEpoch)
{
  ros2_canopen::CommandSession session;
  EXPECT_FALSE(session.accepts(session.epoch()));
  session.revoke();
  const auto old_epoch = session.epoch();
  session.confirm();
  EXPECT_TRUE(session.accepts(old_epoch));
  session.revoke();
  EXPECT_FALSE(session.accepts(old_epoch));
  session.confirm();
  EXPECT_TRUE(session.accepts(session.epoch()));
  EXPECT_FALSE(session.accepts(old_epoch));
}

class TargetMode : public ros2_canopen::ModeTargetHelper<int32_t>
{
public:
  TargetMode() : ModeTargetHelper(8) {}
  bool read(const uint16_t &) override { return true; }
  bool write(OpModeAccesser &) override { return hasTarget(); }
};
TEST(CachedTarget, StartsEmptyAndRestartClearsOldTarget)
{
  TargetMode mode;
  EXPECT_FALSE(mode.hasTarget());
  ASSERT_TRUE(mode.setTarget(123));
  EXPECT_TRUE(mode.hasTarget());
  ASSERT_TRUE(mode.start());
  EXPECT_FALSE(mode.hasTarget());
  ASSERT_TRUE(mode.setTarget(456));
  EXPECT_EQ(mode.getTarget(), 456);
}
TEST(CachedTarget, RejectsNonFiniteValues)
{
  TargetMode mode;
  EXPECT_FALSE(mode.setTarget(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_FALSE(mode.setTarget(std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(mode.setTarget(-std::numeric_limits<double>::infinity()));
  EXPECT_FALSE(mode.hasTarget());
}

TEST(MotorSession, HaltAndRejectedModeRevokeEpochWithoutOpeningCommands)
{
  ros2_canopen::Motor402 motor(nullptr, ros2_canopen::State402::Switched_On, 10);
  const auto initial_epoch = motor.commandEpoch();
  EXPECT_FALSE(motor.commandReady());
  EXPECT_FALSE(motor.setTarget(1, initial_epoch));
  EXPECT_TRUE(motor.handleHalt());  // Unknown feedback: no bridge access or enable request.
  EXPECT_GT(motor.commandEpoch(), initial_epoch);
  EXPECT_FALSE(motor.setTarget(1, initial_epoch));
  const auto halted_epoch = motor.commandEpoch();
  EXPECT_FALSE(motor.enterModeAndWait(ros2_canopen::MotorBase::Homing));
  EXPECT_GT(motor.commandEpoch(), halted_epoch);
  EXPECT_FALSE(motor.commandReady());
  EXPECT_FALSE(motor.enterModeAndWait(33));  // Invalid mode cannot reach OD access or bit shift.
}

#include "canopen_402_driver/ip_target.hpp"
TEST(IpTarget, MirrorsIdenticalSignedTargetAndFailsBeforeRelease)
{
  std::vector<std::tuple<uint16_t, uint8_t, int32_t>> writes;
  ros2_canopen::write_ip_target(-123, true, [&](uint16_t i, uint8_t s, int32_t v) {
    writes.emplace_back(i, s, v);
  });
  ASSERT_EQ(writes.size(), 2u);
  EXPECT_EQ(writes[0], std::make_tuple(uint16_t(0x60C1), uint8_t(1), int32_t(-123)));
  EXPECT_EQ(writes[1], std::make_tuple(uint16_t(0x607A), uint8_t(0), int32_t(-123)));
  EXPECT_THROW(ros2_canopen::write_ip_target(1, true, [](uint16_t, uint8_t, int32_t) {
    throw std::runtime_error("PDO unavailable");
  }), std::runtime_error);
}
#include "canopen_base_driver/motion_health.hpp"
TEST(CommunicationFaults, RecoveryNeverRevalidatesOldTransportEpoch)
{
  using H = ros2_canopen::MotionHealth;
  H health;
  health.configure(true, std::chrono::milliseconds(100));
  auto now = H::Clock::time_point(std::chrono::seconds(1));
  EXPECT_FALSE(health.snapshot(now).ready);
  health.nmt(true);
  auto feed = [&]() { for (auto i : {0x6041, 0x6061, 0x6064, 0x606C}) health.feedback(i, now); };
  feed();
  ASSERT_TRUE(health.snapshot(now).ready);
  auto epoch = health.snapshot(now).epoch;
  health.heartbeat(true);
  EXPECT_FALSE(health.snapshot(now).ready);
  health.heartbeat(false);
  EXPECT_TRUE(health.snapshot(now).ready);
  EXPECT_GT(health.snapshot(now).epoch, epoch);
  epoch = health.snapshot(now).epoch;
  health.emergency(0x2310);
  EXPECT_FALSE(health.snapshot(now).ready);
  health.emergency(0);
  EXPECT_TRUE(health.snapshot(now).ready);
  EXPECT_GT(health.snapshot(now).epoch, epoch);
  epoch = health.snapshot(now).epoch;
  health.nmt(false); health.nmt(true);
  EXPECT_GT(health.snapshot(now).epoch, epoch);
  epoch = health.snapshot(now).epoch;
  now += std::chrono::milliseconds(101);
  feed();  // Gap is detected even without querying readiness while disconnected.
  EXPECT_TRUE(health.snapshot(now).ready);
  EXPECT_GT(health.snapshot(now).epoch, epoch);
  now += std::chrono::milliseconds(101);
  EXPECT_FALSE(health.snapshot(now).ready);
}
TEST(CommunicationFaults, GroupTimingHasWarningStopBoundaryAndStableRecovery) {
  using H=ros2_canopen::MotionHealth;H health;health.configure(true,std::chrono::milliseconds(100));
  for(auto i:{0x6041,0x6061,0x6064,0x606C})health.timing(i,std::chrono::milliseconds(50),std::chrono::milliseconds(100),std::chrono::milliseconds(30));
  auto now=H::Clock::time_point(std::chrono::seconds(1));health.nmt(true);
  auto feed=[&]{for(auto i:{0x6041,0x6061,0x6064,0x606C})health.feedback(i,now);};
  feed();EXPECT_FALSE(health.snapshot(now).ready);
  now+=std::chrono::milliseconds(30);feed();ASSERT_TRUE(health.snapshot(now).ready);
  auto epoch=health.snapshot(now).epoch;now+=std::chrono::milliseconds(50);
  EXPECT_TRUE(health.snapshot(now).ready);EXPECT_TRUE(health.snapshot(now).warning);
  now+=std::chrono::milliseconds(50);EXPECT_FALSE(health.snapshot(now).ready);EXPECT_GT(health.snapshot(now).epoch,epoch);
  feed();EXPECT_FALSE(health.snapshot(now).ready);now+=std::chrono::milliseconds(30);feed();EXPECT_TRUE(health.snapshot(now).ready);
  health.emergency(0x2310);EXPECT_EQ(health.snapshot(now).emcy,0x2310);EXPECT_FALSE(health.snapshot(now).ready);
}
TEST(CommunicationFaults, FeedbackTypesHaveIndependentTrustedDeadlines) {
  using H=ros2_canopen::MotionHealth;H health;health.configure(true,std::chrono::milliseconds(100));health.nmt(true);
  health.timing(0x6064,std::chrono::milliseconds(20),std::chrono::milliseconds(40),std::chrono::milliseconds(0));
  auto now=H::Clock::time_point(std::chrono::seconds(1));
  for(auto i:{0x6041,0x6061,0x6064,0x606C})health.feedback(i,now);
  ASSERT_TRUE(health.snapshot(now).ready);now+=std::chrono::milliseconds(40);
  EXPECT_FALSE(health.snapshot(now).ready);
  EXPECT_THROW(health.timing(0x6064,std::chrono::milliseconds(50),std::chrono::milliseconds(40),std::chrono::milliseconds(0)),std::invalid_argument);
}
