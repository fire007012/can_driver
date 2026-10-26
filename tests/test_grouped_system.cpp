#include <gtest/gtest.h>
#include "can_driver/can_driver_system.hpp"
#include <filesystem>
using can_driver::CanDriverSystem;
using can_driver::SharedDriverState;
using can_driver::SharedDriverSteadyNowNs;
using can_driver::toSystemMotorId;
using R = hardware_interface::return_type;
class FakeProtocol : public CanProtocol {
public:
 std::shared_ptr<SharedDriverState> state;
 std::map<int,int> stops,writes,enables;
 int fail_send=0, fail_enable=0;
 explicit FakeProtocol(std::shared_ptr<SharedDriverState> s):state(s){}
 SharedDriverState::AxisKey key(MotorID id) const {return {"test",CanType::PP,can_driver::NormalizeProtocolMotorId(id)};}
 bool setMode(MotorID id,MotorMode mode) override {
  state->mutateAxisFeedback(key(id),[&](auto f){f->mode=mode;f->modeValid=true;});return true;
 }
 bool setVelocity(MotorID id,int32_t) override {++writes[toSystemMotorId(id)];return toSystemMotorId(id)!=fail_send;}
 bool setPosition(MotorID id,int32_t q) override {return setVelocity(id,q);}
 bool quickSetPosition(MotorID id,int32_t q) override {return setVelocity(id,q);}
 void initializeMotorRefresh(const std::vector<MotorID>&) override {}
 bool setAcceleration(MotorID,int32_t) override {return true;}
 bool setDeceleration(MotorID,int32_t) override {return true;}
 bool Enable(MotorID id) override {
  ++enables[toSystemMotorId(id)];if(toSystemMotorId(id)==fail_enable)return false;
  state->mutateAxisFeedback(key(id),[&](auto f){f->enabled=true;f->enabledValid=true;f->lastRxSteadyNs=SharedDriverSteadyNowNs();f->feedbackSteadyNs.fill(f->lastRxSteadyNs);});return true;
 }
 bool Disable(MotorID) override{return true;}
 bool Stop(MotorID id) override{++stops[toSystemMotorId(id)];return true;}
 int64_t getPosition(MotorID) const override{return 0;}
 int16_t getCurrent(MotorID) const override{return 0;}
 int32_t getVelocity(MotorID) const override{return 0;}
};
class FakeManager : public IDeviceManager {
public:
 std::shared_ptr<SharedDriverState> state=std::make_shared<SharedDriverState>();
 std::shared_ptr<FakeProtocol> protocol=std::make_shared<FakeProtocol>(state);
 bool ensureTransport(const std::string&,bool) override{return true;}
 bool ensureProtocol(const std::string&,CanType) override{return true;}
 bool initDevice(const std::string&,const std::vector<std::pair<CanType,MotorID>> & ids,bool) override {
  for(auto entry:ids) state->mutateAxisFeedback(protocol->key(entry.second),[](auto f){
   f->positionValid=f->velocityValid=f->faultValid=f->feedbackSeen=true;f->fault=false;f->lastRxSteadyNs=SharedDriverSteadyNowNs();f->feedbackSteadyNs.fill(f->lastRxSteadyNs);
  });state->mutateDeviceHealth("test",[](auto h){h->transportReady=true;});return true;
 }
 void startRefresh(const std::string&,CanType,const std::vector<MotorID>&) override{}
 void setRefreshRateHz(double) override{}
 void setDeviceRefreshRateHz(const std::string&,double) override{}
 void setPpFastWriteEnabled(bool) override{}
 void setPpDefaultPositionVelocityRaw(int32_t) override{}
 void setPpPositionDefaultVelocityRaw(int32_t) override{}
 void setPpCspDefaultVelocityRaw(int32_t) override{}
 bool configureDmMotorMasterId(const std::string&,MotorID,uint32_t) override{return true;}
 void shutdownDevice(const std::string&) override{}
 void shutdownAll() override{}
 std::shared_ptr<CanProtocol> getProtocol(const std::string&,CanType) const override{return protocol;}
 std::shared_ptr<std::mutex> getDeviceMutex(const std::string&) const override{return {};}
 bool isDeviceReady(const std::string&) const override{return true;}
 std::shared_ptr<SharedDriverState> getSharedDriverState() const override{return state;}
 size_t deviceCount() const override{return 1;}
};
class GroupedSystem : public ::testing::Test {
protected:
 static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
 static void TearDownTestSuite(){rclcpp::shutdown();}
 std::shared_ptr<FakeManager> manager=std::make_shared<FakeManager>();
 CanDriverSystem system{manager};
 std::shared_ptr<robot_safety::Domain> d;
 std::vector<hardware_interface::CommandInterface> commands;
 std::vector<hardware_interface::StateInterface> states;
 std::shared_ptr<rclcpp::Node> client;
 void SetUp() override {
  hardware_interface::HardwareInfo info;info.name="CanDriverSystem";info.type="system";
  const auto name=std::string("plugin_")+::testing::UnitTest::GetInstance()->current_test_info()->name();
  const auto path=(std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/"robot_safety/config/group_policy_v1.yaml").string();
  info.hardware_parameters={{"safety_domain",name},{"group_policy",path}};
  int id=1;
  for(const auto & name:{"left_front_wheel_joint","right_front_wheel_joint","left_gripper_finger_joint"}) {
   hardware_interface::ComponentInfo joint;joint.name=name;joint.type="joint";
   bool velocity=id<3;
   joint.parameters={{"motor_id",std::to_string(id++)},{"protocol","PP"},{"can_device","test"},{"control_mode",velocity?"velocity":"csp"}};
   hardware_interface::InterfaceInfo cmd;cmd.name=velocity?"velocity":"position";cmd.min="-1";cmd.max="1";
   joint.command_interfaces.push_back(cmd);cmd.name="position";joint.state_interfaces.push_back(cmd);
   info.joints.push_back(joint);
  }
  ASSERT_EQ(system.on_init(info),hardware_interface::CallbackReturn::SUCCESS);
  d=robot_safety::domain(name);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()),hardware_interface::CallbackReturn::SUCCESS);
  commands=system.export_command_interfaces();states=system.export_state_interfaces();
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),hardware_interface::CallbackReturn::SUCCESS);
  cycle();
  const auto until=SharedDriverSteadyNowNs()+30000000000LL;
  d->condition("arm_secured",robot_safety::Domain::Evidence::Satisfied,"commissioned_arm_pose_and_brake",until);
  d->condition("support_trusted",robot_safety::Domain::Evidence::Satisfied,"commissioned_support_state",until);
  d->condition("payload_retained",robot_safety::Domain::Evidence::Satisfied,"commissioned_payload_state",until);
  client=std::make_shared<rclcpp::Node>("plugin_test_client");
 }
 void cycle(){d->renewOwner(1000000000);EXPECT_EQ(system.read(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),R::OK);
  EXPECT_EQ(system.write(rclcpp::Time(0),rclcpp::Duration::from_seconds(.005)),R::OK);}
 void prepare(const std::string & group){
  for (int i=1;i<=3;++i)manager->state->mutateAxisFeedback(manager->protocol->key(static_cast<MotorID>(i)),[](auto f){f->positionValid=f->velocityValid=f->faultValid=f->feedbackSeen=true;f->fault=false;f->lastRxSteadyNs=SharedDriverSteadyNowNs();f->feedbackSteadyNs.fill(f->lastRxSteadyNs);});
  auto service=client->create_client<std_srvs::srv::Trigger>("/robot_backend/CanDriverSystem/"+group+"/prepare");
  ASSERT_TRUE(service->wait_for_service(std::chrono::seconds(3)));
  auto result=service->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  ASSERT_EQ(rclcpp::spin_until_future_complete(client,result,std::chrono::seconds(5)),rclcpp::FutureReturnCode::SUCCESS);
  ASSERT_TRUE(result.get()->success);
  const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  do {cycle();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
  while(!d->group(group).ready && std::chrono::steady_clock::now()<deadline);
  ASSERT_TRUE(d->group(group).ready);
  for(const auto & e:d->events())if(e.group==group)d->acknowledge(e.id);
  ASSERT_TRUE(d->arm(group));
 }
 void start(){prepare("base");prepare("gripper");
  ASSERT_EQ(system.perform_command_mode_switch({"left_front_wheel_joint/velocity","right_front_wheel_joint/velocity","left_gripper_finger_joint/position"},{}),R::OK);
  ASSERT_TRUE(d->run("base"));ASSERT_TRUE(d->run("gripper"));
  commands[0].set_value(.2);commands[1].set_value(.2);commands[2].set_value(.02);cycle();
 }
};
TEST_F(GroupedSystem, TrackFeedbackFaultKeepsGripperAndInterfacesAvailable){
 start();auto epoch=d->epoch("gripper");auto writes=manager->protocol->writes[3];
 manager->state->mutateAxisFeedback(manager->protocol->key(static_cast<MotorID>(1)),[](auto f){f->fault=true;});
 cycle();EXPECT_FALSE(d->permits("base",d->epoch("base")));EXPECT_TRUE(d->permits("gripper",epoch));
 EXPECT_GT(manager->protocol->writes[3],writes);EXPECT_GT(manager->protocol->stops[1],0);EXPECT_GT(manager->protocol->stops[2],0);
 EXPECT_EQ(system.export_command_interfaces().size(),3u);
 EXPECT_EQ(system.perform_command_mode_switch({}, {"left_front_wheel_joint/velocity","right_front_wheel_joint/velocity"}),R::OK);
 EXPECT_EQ(d->epoch("gripper"),epoch);
}
TEST_F(GroupedSystem, InvalidOutputIsRejectedBeforeEitherTrackWrites){
 start();auto left=manager->protocol->writes[1],right=manager->protocol->writes[2],grip=manager->protocol->writes[3];
 commands[1].set_value(INFINITY);cycle();
 EXPECT_EQ(manager->protocol->writes[1],left);EXPECT_EQ(manager->protocol->writes[2],right);
 EXPECT_GT(manager->protocol->writes[3],grip);
}
TEST_F(GroupedSystem, SendFailureClosesAllGroupsOnTheSharedPrivateChannel){
 start();auto flippers=d->epoch("flippers");manager->protocol->fail_send=1;cycle();
 EXPECT_EQ(d->group("base").state,robot_safety::Domain::State::Fault);
 EXPECT_EQ(d->group("gripper").state,robot_safety::Domain::State::Fault);
 EXPECT_EQ(d->epoch("flippers"),flippers);
}
TEST_F(GroupedSystem, SharedTransportMetadataFailureDoesNotWaitForAxisFeedbackExpiry){
 start();auto writes=manager->protocol->writes[3];
 manager->state->mutateDeviceHealth("test",[](auto h){++h->txError;});cycle();
 EXPECT_EQ(d->group("base").state,robot_safety::Domain::State::Fault);
 EXPECT_EQ(d->group("gripper").state,robot_safety::Domain::State::Fault);
 EXPECT_EQ(manager->protocol->writes[3],writes);
}
TEST_F(GroupedSystem, FreshUnrelatedReplyCannotRefreshAnOldPositionField){
 start();auto writes=manager->protocol->writes[1];
 manager->state->mutateAxisFeedback(manager->protocol->key(static_cast<MotorID>(1)),[](auto f){
  f->lastRxSteadyNs=SharedDriverSteadyNowNs();f->feedbackSteadyNs[0]=f->lastRxSteadyNs-500000000;
 });cycle();EXPECT_EQ(d->group("base").state,robot_safety::Domain::State::Fault);
 EXPECT_EQ(manager->protocol->writes[1],writes);
}
