#include <gtest/gtest.h>
#include "canopen_402_driver/node_interfaces/node_canopen_402_driver.hpp"
class ManagedDriver : public ros2_canopen::node_interfaces::NodeCanopen402Driver<rclcpp::Node> {
public:
 explicit ManagedDriver(rclcpp::Node *node):NodeCanopen402Driver(node){managed_by_ros2_control_=true;}
};
TEST(ManagedCommands, OnlyHardwareInterfaceCanStartMotion)
{
 rclcpp::init(0,nullptr);
 {
  rclcpp::Node node("managed_command_test");ManagedDriver driver(&node);
  auto request=std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response=std::make_shared<std_srvs::srv::Trigger::Response>();
  driver.handle_init(request,response);EXPECT_FALSE(response->success);
  driver.handle_enable(request,response);EXPECT_FALSE(response->success);
  driver.handle_set_mode_cyclic_position(request,response);EXPECT_FALSE(response->success);
  driver.handle_set_mode_interpolated_position(request,response);EXPECT_FALSE(response->success);
  auto target=std::make_shared<canopen_interfaces::srv::COTargetDouble::Request>();
  auto result=std::make_shared<canopen_interfaces::srv::COTargetDouble::Response>();
  driver.handle_set_target(target,result);EXPECT_FALSE(result->success);
  ros2_canopen::COData data{0x607A,0,123};EXPECT_FALSE(driver.tpdo_transmit(data));EXPECT_FALSE(driver.sdo_write(data));
 }
 rclcpp::shutdown();
}
