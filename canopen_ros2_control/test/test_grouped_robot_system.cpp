#include <gtest/gtest.h>
#include "canopen_ros2_control/robot_system.hpp"
#include "robot_safety/policy.hpp"
#include <filesystem>
using D=robot_safety::Domain;
using R=hardware_interface::return_type;
class AxisDriver : public ros2_canopen::Cia402Driver {
public:
 bool communication=true,command=true,warning=false;
 uint16_t mode=ros2_canopen::MotorBase::Cyclic_Synchronous_Position,emcy=0;
 uint64_t epoch=1;int writes=0,halts=0;double q=0;
 double get_speed() override{return 0;}
 double get_position() override{return q;}
 bool communication_ready() override{return communication;}
 ros2_canopen::MotionHealth::Snapshot communication_status() override{return {communication,epoch,emcy?"EMCY":"feedback",warning,emcy};}
 bool command_ready() override{return command&&communication;}
 uint64_t command_epoch() override{return epoch;}
 bool set_operation_mode(uint16_t value) override{++epoch;mode=value;command=true;return communication;}
 uint16_t get_mode() override{return mode;}
 bool set_target(double,uint64_t e) override{if(e!=epoch||!command_ready())return false;++writes;return true;}
 bool halt_motor() override{++halts;++epoch;command=false;return true;}
 bool init_motor() override{command=true;return communication;}
};
class System : public canopen_ros2_control::RobotSystem {
public:
 void setup(const std::shared_ptr<D> & d) {
  safety_=d;grouped_=true;configured_ok_=true;hardware_active_=true;info_.name="RobotCanopenSystem";
  for(const auto & name:{"shoulder_yaw_joint","shoulder_pitch_joint","left_front_arm_joint","right_front_arm_joint","left_rear_arm_joint","right_rear_arm_joint"}) {
   canopen_ros2_control::Cia402Data data;data.joint_name=name;data.session.group=d->axisGroup(name,info_.name);
   data.session.prepared->store(true);data.session.prepared_epoch->store(d->epoch(data.session.group));
   data.driver=std::make_shared<AxisDriver>();
   data.command_interface_to_operation_mode[std::string(name)+"/position"]=ros2_canopen::MotorBase::Cyclic_Synchronous_Position;
   if(data.session.group=="flippers")data.command_interface_to_operation_mode[std::string(name)+"/velocity"]=ros2_canopen::MotorBase::Cyclic_Synchronous_Velocity;
   robot_motor_data_.push_back(data);
  }
  export_command_interfaces();
 }
 auto & axes(){return robot_motor_data_;}
};
class GroupedRobotSystem : public ::testing::Test {
protected:
 static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
 std::shared_ptr<D> d=std::make_shared<D>();System system;
 AxisDriver & driver(size_t i){return *std::static_pointer_cast<AxisDriver>(system.axes()[i].driver);}
 void SetUp() override {
  auto path=std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path()/"robot_safety/config/group_policy_v1.yaml";
  auto p=robot_safety::Policy::load(path.string());d->configure(p.axes,p.associations,p.version);
  for(const auto & a:p.axes)d->reportAxis(a.first,true);
  system.setup(d);
  EXPECT_EQ(system.read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),R::OK);
  ASSERT_TRUE(d->arm("arm"));ASSERT_TRUE(d->arm("flippers"));
  std::vector<std::string> start;
  for(const auto & a:system.axes())start.push_back(a.joint_name+(a.session.group=="arm"?"/position":"/velocity"));
  ASSERT_EQ(system.perform_command_mode_switch(start,{}),R::OK);
  ASSERT_TRUE(d->run("arm"));ASSERT_TRUE(d->run("flippers"));
  for(auto & a:system.axes()){a.target_position=.02;a.target_velocity=.1;}
  write();
 }
 void write(){EXPECT_EQ(system.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),R::OK);}
};
TEST_F(GroupedRobotSystem, PrivateArmFaultClosesBothShouldersWithoutStoppingFlippers) {
 auto epoch=d->epoch("flippers");int writes=driver(2).writes;
 d->reportAxis("elbow_pitch_joint",false,"private axis offline");write();
 EXPECT_GT(driver(0).halts,0);EXPECT_GT(driver(1).halts,0);EXPECT_EQ(driver(2).writes,writes+1);
 EXPECT_TRUE(d->permits("flippers",epoch));EXPECT_EQ(d->epoch("flippers"),epoch);
}
TEST_F(GroupedRobotSystem, MissingDriverKeepsOtherStateInterfacesAndReleasesStops) {
 system.axes()[0].driver.reset();
 EXPECT_EQ(system.read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),R::OK);
 EXPECT_TRUE(std::isnan(system.axes()[0].actual_position));EXPECT_TRUE(std::isfinite(system.axes()[2].actual_position));
 write();
 EXPECT_EQ(system.perform_command_mode_switch({}, {"shoulder_yaw_joint/position","shoulder_pitch_joint/position"}),R::OK);
 EXPECT_EQ(system.export_state_interfaces().size(),12u);
}
TEST_F(GroupedRobotSystem, UnknownEmcyPreservesOriginalCodeAndStopsWholeArm) {
 driver(0).communication=false;
 EXPECT_EQ(system.read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),R::OK);write();
 driver(0).emcy=0x2310;
 EXPECT_EQ(system.read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),R::OK);write();
 bool recorded=false;for(const auto & e:d->events())if(e.axis=="shoulder_yaw_joint"&&e.code=="8976"&&e.category=="unknown_emcy")recorded=true;
 EXPECT_TRUE(recorded);EXPECT_TRUE(system.axes()[0].interfaces_to_running.empty());EXPECT_TRUE(system.axes()[1].interfaces_to_running.empty());
 EXPECT_EQ(d->group("flippers").state,D::State::Running);
}
TEST_F(GroupedRobotSystem, PreflightRejectsSecondShoulderBeforeFirstShoulderWrites) {
 int first=driver(0).writes,second=driver(1).writes,flipper=driver(2).writes;
 system.axes()[1].target_position=INFINITY;write();
 EXPECT_EQ(driver(0).writes,first);EXPECT_EQ(driver(1).writes,second);EXPECT_EQ(driver(2).writes,flipper+1);
}
TEST_F(GroupedRobotSystem, FlipperModeSwitchLeavesLiveArmEpochAndTargetsIntact) {
 auto arm=d->epoch("arm");auto command=system.axes()[0].target_position;
 d->invalidate("flippers");
 std::vector<std::string> stop,start;
 for(size_t i=2;i<system.axes().size();++i){stop.push_back(system.axes()[i].joint_name+"/velocity");start.push_back(system.axes()[i].joint_name+"/position");}
 ASSERT_EQ(system.perform_command_mode_switch({},stop),R::OK);
 for(size_t i=2;i<system.axes().size();++i){system.axes()[i].session.prepared->store(true);system.axes()[i].session.prepared_epoch->store(d->epoch("flippers"));}
 ASSERT_TRUE(d->arm("flippers"));ASSERT_EQ(system.perform_command_mode_switch(start,{}),R::OK);
 EXPECT_EQ(d->epoch("arm"),arm);EXPECT_EQ(system.axes()[0].target_position,command);EXPECT_TRUE(d->permits("arm",arm));
}
