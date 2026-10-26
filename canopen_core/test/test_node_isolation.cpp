#include <gtest/gtest.h>
#include "canopen_core/device_container.hpp"
#include "canopen_core/driver_error.hpp"
#include "canopen_core/node_interfaces/node_canopen_driver.hpp"
#include <filesystem>
using namespace ros2_canopen;
class Interface : public ros2_canopen::node_interfaces::NodeCanopenDriver<rclcpp::Node> {
public:
 explicit Interface(rclcpp::Node *node):NodeCanopenDriver(node){}
 int initialized=0,configured=0,activated=0;bool fail=false;
 void init() override{++initialized;}
 void configure() override{++configured;}
 void activate() override{++activated;if(fail)throw AxisBootException("axis boot timeout");}
};
class Driver : public CanopenDriver {
public:
 std::shared_ptr<Interface> interface=std::make_shared<Interface>(this);
 int inits=0;bool bad_config=false;
 Driver(){node_canopen_driver_=interface;}
 void init() override{++inits;if(bad_config)throw DriverException("invalid axis configuration");if(interface->fail)throw AxisBootException("axis boot timeout");}
 void shutdown() override{}
 void set_master(std::shared_ptr<lely::ev::Executor>,std::shared_ptr<lely::canopen::AsyncMaster>) override{}
};
class Master : public CanopenMasterInterface, public rclcpp::Node {
public:
 Master():Node("isolated_master"){}
 void init() override{}
 void shutdown() override{}
 bool is_lifecycle() override{return false;}
 std::shared_ptr<lely::ev::Executor> get_executor() override{return {};}
 std::shared_ptr<lely::canopen::AsyncMaster> get_master() override{return {};}
 rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() override{return Node::get_node_base_interface();}
};
class Container : public DeviceContainer {
public:
 bool bad_config=false;
 Container(){
  auto path=std::filesystem::path(__FILE__).parent_path()/"bus_configs/good_master_and_two_driver.yml";
  auto file=path.string();config_=std::make_shared<ConfigurationManager>(file);config_->init_config();
  can_master_=std::make_shared<Master>();can_master_id_=1;
 }
 bool load_component(const std::string,const std::string,const uint16_t id,const std::string,
   std::vector<rclcpp::Parameter>&,const std::string) override {
  auto d=std::make_shared<Driver>();d->interface->fail=id==2;d->bad_config=bad_config&&id==2;registered_drivers_[id]=d;return true;
 }
 void add_node_to_executor(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr) override{}
 bool load(){return load_drivers();}
 Driver & driver(uint16_t id){return *std::static_pointer_cast<Driver>(registered_drivers_.at(id));}
};
class NodeIsolation : public ::testing::Test {
protected:
 static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
};
TEST_F(NodeIsolation, DefaultRetainsLegacyAllOrNothingBootPolicy){
 Container c;EXPECT_THROW(c.load(),AxisBootException);EXPECT_EQ(c.count_drivers(),1u);
}
TEST_F(NodeIsolation, AxisBootFailureContinuesHealthyDriversOnTheSameMaster){
 Container c;c.allow_partial_drivers(true);EXPECT_TRUE(c.load());EXPECT_EQ(c.count_drivers(),2u);
 EXPECT_EQ(c.failed_drivers().at(2),"axis boot timeout");EXPECT_EQ(c.driver(3).inits,1);
}
TEST_F(NodeIsolation, InvalidConfigurationIsNeverSilentlyDowngradedToAnOfflineAxis){
 Container c;c.allow_partial_drivers(true);c.bad_config=true;EXPECT_THROW(c.load(),DriverException);EXPECT_TRUE(c.failed_drivers().empty());
}
TEST_F(NodeIsolation, LazyActivationBootsOnlyTheSelectedAxisAndAllowsBoundedRetry){
 Container c;c.allow_partial_drivers(true);c.lazy_driver_activation(true);ASSERT_TRUE(c.load());
 EXPECT_EQ(c.driver(2).inits,0);EXPECT_EQ(c.driver(3).inits,0);
 EXPECT_EQ(c.driver(2).interface->configured,1);EXPECT_EQ(c.driver(3).interface->activated,0);
 EXPECT_FALSE(c.retry_driver(2));EXPECT_TRUE(c.retry_driver(3));EXPECT_TRUE(c.retry_driver(3));
 EXPECT_EQ(c.driver(3).interface->activated,1);c.driver(2).interface->fail=false;
 EXPECT_TRUE(c.retry_driver(2));EXPECT_TRUE(c.failed_drivers().empty());
}
