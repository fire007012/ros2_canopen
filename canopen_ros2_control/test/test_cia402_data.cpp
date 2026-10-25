#include <gtest/gtest.h>
#include "canopen_ros2_control/cia402_data.hpp"

using canopen_ros2_control::Cia402Data;
using ros2_canopen::MotorBase;
struct Motor
{
  uint16_t mode = MotorBase::Cyclic_Synchronous_Position;
  bool switch_ok = true, halt_ok = true, ready = true, feedback_ok = true;
  uint64_t epoch = 0;
  int writes = 0, halts = 0;
  double target = 0;
  uint16_t get_mode() { return mode; }
  bool command_ready() { return ready; }
  uint64_t command_epoch() { return epoch; }
  bool set_operation_mode(uint16_t value)
  { ++epoch; if (!switch_ok) return false; mode = value; ready = feedback_ok; return true; }
  bool halt_motor() { ++halts; ++epoch; ready = false; return halt_ok; }
  bool set_target(double value, uint64_t session)
  { if (!ready || session != epoch) return false; ++writes; target = value; return true; }
};
class RobotCommandSession : public testing::Test
{
protected:
  Cia402Data data;
  Motor motor;
  void SetUp() override
  {
    data.command_interface_to_operation_mode = {
      {"joint/position", MotorBase::Cyclic_Synchronous_Position},
      {"joint/velocity", MotorBase::Cyclic_Synchronous_Velocity},
      {"joint/ip", MotorBase::Interpolated_Position}};
  }
  bool start(const std::string & name)
  { data.interfaces_to_start = {name}; return data.perform_switch(motor); }
};
TEST_F(RobotCommandSession, SameModeStartRequiresFreshFiniteCommand)
{
  data.target_position = 123;
  ASSERT_TRUE(start("joint/position"));
  data.write_target(motor);
  EXPECT_EQ(motor.writes, 0);
  data.target_position = 0.5;
  data.write_target(motor);
  EXPECT_EQ(motor.target, 0.5);
  data.target_position = INFINITY;
  data.write_target(motor);
  EXPECT_EQ(motor.writes, 1);
}
TEST_F(RobotCommandSession, StopOnlyHaltsAndRevokesCommands)
{
  ASSERT_TRUE(start("joint/position"));
  data.target_position = 1;
  data.interfaces_to_stop = {"joint/position"};
  ASSERT_TRUE(data.perform_switch(motor));
  EXPECT_EQ(motor.halts, 1);
  EXPECT_TRUE(data.interfaces_to_running.empty());
  EXPECT_TRUE(data.perform_switch(motor));
  EXPECT_EQ(motor.halts, 1);
  data.write_target(motor);
  EXPECT_EQ(motor.writes, 0);
}
TEST_F(RobotCommandSession, FailedSwitchDoesNotPublishRunningOrReplay)
{
  ASSERT_TRUE(start("joint/position"));
  data.interfaces_to_stop = {"joint/position"};
  motor.switch_ok = false;
  EXPECT_FALSE(start("joint/velocity"));
  EXPECT_TRUE(data.interfaces_to_running.empty());
  EXPECT_TRUE(data.interfaces_to_start.empty());
  data.write_target(motor);
  EXPECT_EQ(motor.writes, 0);
}
TEST_F(RobotCommandSession, PositionVelocityAndIpSwitches)
{
  ASSERT_TRUE(start("joint/position"));
  data.interfaces_to_stop = {"joint/position"};
  ASSERT_TRUE(start("joint/velocity"));
  data.target_velocity = 0.2;
  data.write_target(motor);
  EXPECT_EQ(motor.target, 0.2);
  data.interfaces_to_stop = {"joint/velocity"};
  ASSERT_TRUE(start("joint/ip"));
  data.target_position = 0.8;
  data.write_target(motor);
  EXPECT_EQ(motor.target, 0.8);
  motor.mode = MotorBase::No_Mode;
  data.write_target(motor);
  EXPECT_EQ(motor.writes, 2);
}
TEST_F(RobotCommandSession, RejectsUnknownMultipleAndConflictingStarts)
{
  EXPECT_FALSE(start("unknown"));
  data.interfaces_to_start = {"joint/position", "joint/velocity"};
  EXPECT_FALSE(data.perform_switch(motor));
  ASSERT_TRUE(start("joint/position"));
  EXPECT_FALSE(start("joint/velocity"));
  EXPECT_EQ(data.interfaces_to_running.front(), "joint/position");
}
TEST_F(RobotCommandSession, HaltFailureKeepsSessionClosed)
{
  ASSERT_TRUE(start("joint/position"));
  motor.halt_ok = false;
  data.interfaces_to_stop = {"joint/position"};
  EXPECT_FALSE(start("joint/velocity"));
  EXPECT_TRUE(data.interfaces_to_running.empty());
}

TEST_F(RobotCommandSession, StopAndSameModeRestartNeedsConfirmedStateAndFreshTarget)
{
  ASSERT_TRUE(start("joint/position"));
  data.target_position = 123;
  data.interfaces_to_stop = {"joint/position"};
  ASSERT_TRUE(data.perform_switch(motor));
  motor.feedback_ok = false;
  EXPECT_FALSE(start("joint/position"));
  EXPECT_TRUE(data.interfaces_to_running.empty());
  motor.feedback_ok = true;
  ASSERT_TRUE(start("joint/position"));
  EXPECT_TRUE(data.write_target(motor));
  EXPECT_EQ(motor.writes, 0);
  data.target_position = 0.5;
  EXPECT_TRUE(data.write_target(motor));
  EXPECT_EQ(motor.target, 0.5);
}
TEST_F(RobotCommandSession, FaultAndRecoveryBetweenWritesRevokesOldSession)
{
  ASSERT_TRUE(start("joint/position"));
  data.target_position = 12;
  ++motor.epoch;  // Fault was observed and then recovered between control cycles.
  EXPECT_FALSE(data.write_target(motor));
  EXPECT_TRUE(data.interfaces_to_running.empty());
  data.target_position = 12;  // The still-active controller cannot reopen its old session.
  EXPECT_TRUE(data.write_target(motor));
  EXPECT_EQ(motor.writes, 0);
  ASSERT_TRUE(start("joint/position"));
  EXPECT_TRUE(std::isnan(data.target_position));
}
TEST_F(RobotCommandSession, LostFeedbackClosesSession)
{
  ASSERT_TRUE(start("joint/position"));
  data.target_position = 12;
  motor.ready = false;
  EXPECT_FALSE(data.write_target(motor));
  EXPECT_TRUE(data.interfaces_to_running.empty());
  motor.ready = true;
  data.write_target(motor);
  EXPECT_EQ(motor.writes, 0);
}
TEST_F(RobotCommandSession, LimitsAndInvalidCommandAfterMotionFailClosed)
{
  data.lower=-.1;data.upper=.1;
  ASSERT_TRUE(start("joint/position"));
  data.target_position=.05;ASSERT_TRUE(data.write_target(motor));
  data.target_position=NAN;EXPECT_FALSE(data.write_target(motor));
  data.target_position=.2;EXPECT_FALSE(data.write_target(motor));
  EXPECT_EQ(motor.writes,1);
}
TEST(DeclaredInterfaces, SingleAxisExportsOnlyItsUrdfContract)
{
  hardware_interface::ComponentInfo joint;joint.name="axis";
  joint.command_interfaces.emplace_back();joint.command_interfaces.back().name="velocity";
  Cia402Data data;ASSERT_TRUE(data.init_data(joint,"node_id: 1\nposition_mode: 8\nvelocity_mode: 9\n"));
  std::vector<hardware_interface::CommandInterface> interfaces;data.export_command_interface(interfaces);
  ASSERT_EQ(interfaces.size(),1u);EXPECT_EQ(interfaces[0].get_name(),"axis/velocity");
}
