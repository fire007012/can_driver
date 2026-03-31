#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/Float64.h>

#include "can_driver/CanDriverHW.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/MotorState.h"
#include "can_driver/lifecycle_service_gateway.hpp"
#include "can_driver/MotorCommand.h"
#include "can_driver/Recover.h"

#include <atomic>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

class FakeProtocol : public CanProtocol {
public:
    bool setMode(MotorID motorId, MotorMode mode) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastModeMotor_ = motorId;
        lastMode_ = mode;
        ++setModeCalls_;
        return true;
    }

    bool setVelocity(MotorID motorId, int32_t velocity) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastVelocityMotor_ = motorId;
        lastVelocity_ = velocity;
        ++setVelocityCalls_;
        return true;
    }

    bool setAcceleration(MotorID, int32_t) override { return true; }
    bool setDeceleration(MotorID, int32_t) override { return true; }

    bool setPosition(MotorID motorId, int32_t position) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastPositionMotor_ = motorId;
        lastPosition_ = position;
        ++setPositionCalls_;
        return true;
    }

    bool quickSetPosition(MotorID motorId, int32_t position) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastQuickPositionMotor_ = motorId;
        lastQuickPosition_ = position;
        ++quickSetPositionCalls_;
        return true;
    }

    bool Enable(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        lastEnableMotor_ = motorId;
        enabled_ = true;
        ++enableCalls_;
        return true;
    }

    bool Disable(MotorID) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        enabled_ = false;
        return true;
    }

    bool Stop(MotorID) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stopCalls_;
        return true;
    }

    int64_t getPosition(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return feedbackPosition_;
    }

    int16_t getCurrent(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return feedbackCurrent_;
    }

    int16_t getVelocity(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return feedbackVelocity_;
    }

    void initializeMotorRefresh(const std::vector<MotorID> &) override {}

    int setModeCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return setModeCalls_;
    }

    uint16_t lastModeMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastModeMotor_);
    }

    MotorMode lastMode() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastMode_;
    }

    int velocityCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return setVelocityCalls_;
    }

    int32_t lastVelocity() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastVelocity_;
    }

    uint16_t lastVelocityMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastVelocityMotor_);
    }

    int enableCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return enableCalls_;
    }

    int positionCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return setPositionCalls_;
    }

    int quickPositionCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return quickSetPositionCalls_;
    }

    int32_t lastQuickPosition() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lastQuickPosition_;
    }

    uint16_t lastQuickPositionMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastQuickPositionMotor_);
    }

    uint16_t lastEnableMotor() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return static_cast<uint16_t>(lastEnableMotor_);
    }

    [[nodiscard]] bool isEnabled(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return enabled_;
    }

    [[nodiscard]] bool hasFault(MotorID) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return hasFault_;
    }

    void setFault(bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        hasFault_ = value;
    }

    void setFeedbackPosition(int64_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        feedbackPosition_ = value;
    }

    void setFeedbackVelocity(int16_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        feedbackVelocity_ = value;
    }

    void setFeedbackCurrent(int16_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        feedbackCurrent_ = value;
    }

    int stopCalls() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return stopCalls_;
    }

private:
    mutable std::mutex mutex_;
    MotorID lastModeMotor_{MotorID::LeftWheel};
    MotorMode lastMode_{MotorMode::Position};
    int setModeCalls_{0};

    MotorID lastVelocityMotor_{MotorID::LeftWheel};
    int32_t lastVelocity_{0};
    int setVelocityCalls_{0};

    MotorID lastPositionMotor_{MotorID::LeftWheel};
    int32_t lastPosition_{0};
    int setPositionCalls_{0};

    MotorID lastQuickPositionMotor_{MotorID::LeftWheel};
    int32_t lastQuickPosition_{0};
    int quickSetPositionCalls_{0};

    MotorID lastEnableMotor_{MotorID::LeftWheel};
    int enableCalls_{0};
    int stopCalls_{0};
    bool enabled_{true};
    bool hasFault_{false};
    int64_t feedbackPosition_{0};
    int16_t feedbackVelocity_{0};
    int16_t feedbackCurrent_{0};
};

class FakeDeviceManager : public IDeviceManager {
public:
    explicit FakeDeviceManager(bool exposeSharedState = false)
        : protocol_(std::make_shared<FakeProtocol>())
        , mutex_(std::make_shared<std::mutex>())
        , sharedState_(std::make_shared<can_driver::SharedDriverState>())
        , exposeSharedState_(exposeSharedState)
    {
    }

    bool ensureTransport(const std::string &, bool = false) override { return true; }
    bool ensureProtocol(const std::string &, CanType) override { return true; }

    bool initDevice(const std::string &,
                    const std::vector<std::pair<CanType, MotorID>> &,
                    bool = false) override
    {
        return true;
    }

    void startRefresh(const std::string &, CanType, const std::vector<MotorID> &) override {}
    void setRefreshRateHz(double) override {}
    void setPpFastWriteEnabled(bool) override {}
    void shutdownAll() override {}

    std::shared_ptr<CanProtocol> getProtocol(const std::string &, CanType) const override
    {
        return protocol_;
    }

    std::shared_ptr<std::mutex> getDeviceMutex(const std::string &) const override
    {
        return mutex_;
    }

    bool isDeviceReady(const std::string &) const override
    {
        std::lock_guard<std::mutex> lock(readyMutex_);
        return ready_;
    }
    std::size_t deviceCount() const override { return 1; }
    std::shared_ptr<can_driver::SharedDriverState> getSharedDriverState() const override
    {
        return exposeSharedState_ ? sharedState_ : nullptr;
    }

    std::shared_ptr<FakeProtocol> protocol() const { return protocol_; }
    std::shared_ptr<can_driver::SharedDriverState> sharedState() const { return sharedState_; }
    void setReady(bool ready)
    {
        std::lock_guard<std::mutex> lock(readyMutex_);
        ready_ = ready;
    }

private:
    std::shared_ptr<FakeProtocol> protocol_;
    std::shared_ptr<std::mutex> mutex_;
    std::shared_ptr<can_driver::SharedDriverState> sharedState_;
    bool exposeSharedState_{false};
    mutable std::mutex readyMutex_;
    bool ready_{true};
};

class CanDriverHWSmokeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        if (!ros::isInitialized()) {
            int argc = 0;
            char **argv = nullptr;
            ros::init(argc, argv, "test_can_driver_hw_smoke",
                      ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
        }
        ros::Time::init();
    }

    static XmlRpc::XmlRpcValue makeSingleVelocityJoint()
    {
        XmlRpc::XmlRpcValue joints;
        joints.setSize(1);
        joints[0]["name"] = "test_wheel";
        joints[0]["motor_id"] = static_cast<int>(0x141);
        joints[0]["protocol"] = "MT";
        joints[0]["can_device"] = "fake0";
        joints[0]["control_mode"] = "velocity";
        joints[0]["velocity_scale"] = 0.1;
        joints[0]["position_scale"] = 1.0;
        return joints;
    }

    static XmlRpc::XmlRpcValue makeSingleCspJoint()
    {
        XmlRpc::XmlRpcValue joints;
        joints.setSize(1);
        joints[0]["name"] = "test_arm";
        joints[0]["motor_id"] = 0x05;
        joints[0]["protocol"] = "PP";
        joints[0]["can_device"] = "fake0";
        joints[0]["control_mode"] = "csp";
        joints[0]["position_scale"] = 65536;
        joints[0]["velocity_scale"] = 65536;
        return joints;
    }

    static std::string uniqueNs(const std::string &base)
    {
        static std::atomic<int> seq{0};
        return "/" + base + "_" + std::to_string(seq.fetch_add(1));
    }

    static void enterRunning(CanDriverHW &hw)
    {
        auto &coordinator = hw.operationalCoordinator();
        const auto initResult = coordinator.RequestInit("fake0", false);
        ASSERT_TRUE(initResult.ok) << initResult.message;

        const auto releaseResult = coordinator.RequestRelease();
        ASSERT_TRUE(releaseResult.ok) << releaseResult.message;
    }
};

TEST_F(CanDriverHWSmokeTest, InitAndDirectWriteUsesProtocol)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_write"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_wheel/cmd_velocity");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);

    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 1.2;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastVelocityMotor(), 0x141u);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 12);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, MotorCommandServiceEnable)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_service"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string srvName = pnh.resolveName("motor_command");
    ros::ServiceClient client = nh.serviceClient<can_driver::MotorCommand>(srvName);
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    can_driver::MotorCommand srv;
    srv.request.motor_id = 0x141;
    srv.request.command = can_driver::MotorCommand::Request::CMD_ENABLE;
    srv.request.value = 0.0;

    ASSERT_TRUE(client.call(srv));
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(srv.response.message, "Driver inactive.");
    EXPECT_EQ(fakeDm->protocol()->enableCalls(), 0);

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;

    ASSERT_TRUE(client.call(srv));
    EXPECT_TRUE(srv.response.success);
    EXPECT_EQ(fakeDm->protocol()->enableCalls(), 2);
    EXPECT_EQ(fakeDm->protocol()->lastEnableMotor(), 0x141u);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, RecoverServiceRejectsPerMotorLegacyRequest)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_recover_service"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));

    can_driver::OperationalCoordinator::DriverOps ops;
    ops.recover_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "recovered"};
    };
    hw.operationalCoordinator().SetDriverOps(std::move(ops));
    hw.operationalCoordinator().SetFaulted();

    LifecycleServiceGateway gateway(pnh, &hw.operationalCoordinator());

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string srvName = pnh.resolveName("recover");
    ros::ServiceClient client = nh.serviceClient<can_driver::Recover>(srvName);
    ASSERT_TRUE(client.waitForExistence(ros::Duration(1.0)));

    can_driver::Recover srv;
    srv.request.motor_id = 0x141;

    ASSERT_TRUE(client.call(srv));
    EXPECT_FALSE(srv.response.success);
    EXPECT_EQ(srv.response.message,
              "per-motor recover has been removed; use motor_id=65535 for global recover");

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, LifecycleServicesExposeCanonicalMessages)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_lifecycle_messages"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("motor_state_period_sec", 0.2);

    ASSERT_TRUE(hw.init(nh, pnh));

    can_driver::OperationalCoordinator::DriverOps ops;
    ops.init_device = [](const std::string &, bool) {
        return can_driver::OperationalCoordinator::Result{true, "driver init detail"};
    };
    ops.enable_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "driver enable detail"};
    };
    ops.disable_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "OK"};
    };
    ops.halt_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "OK"};
    };
    ops.recover_all = []() {
        return can_driver::OperationalCoordinator::Result{true, "driver recover detail"};
    };
    ops.shutdown_all = [](bool) {
        return can_driver::OperationalCoordinator::Result{true, "driver shutdown detail"};
    };
    ops.motion_healthy = [](std::string *) {
        return true;
    };
    ops.any_fault_active = []() {
        return false;
    };
    ops.hold_commands = []() {};
    ops.arm_fresh_command_latch = []() {};

    auto &coordinator = hw.operationalCoordinator();
    coordinator.SetDriverOps(std::move(ops));
    coordinator.SetConfigured();

    LifecycleServiceGateway gateway(pnh, &coordinator);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::ServiceClient initClient = nh.serviceClient<can_driver::Init>(pnh.resolveName("init"));
    ros::ServiceClient enableClient =
        nh.serviceClient<std_srvs::Trigger>(pnh.resolveName("enable"));
    ros::ServiceClient disableClient =
        nh.serviceClient<std_srvs::Trigger>(pnh.resolveName("disable"));
    ros::ServiceClient recoverClient =
        nh.serviceClient<can_driver::Recover>(pnh.resolveName("recover"));
    ros::ServiceClient resumeClient =
        nh.serviceClient<std_srvs::Trigger>(pnh.resolveName("resume"));
    ros::ServiceClient haltClient =
        nh.serviceClient<std_srvs::Trigger>(pnh.resolveName("halt"));
    ros::ServiceClient shutdownClient =
        nh.serviceClient<can_driver::Shutdown>(pnh.resolveName("shutdown"));

    ASSERT_TRUE(initClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(enableClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(disableClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(recoverClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(resumeClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(haltClient.waitForExistence(ros::Duration(1.0)));
    ASSERT_TRUE(shutdownClient.waitForExistence(ros::Duration(1.0)));

    can_driver::Init initSrv;
    initSrv.request.device = "fake0";
    initSrv.request.loopback = false;
    ASSERT_TRUE(initClient.call(initSrv));
    EXPECT_TRUE(initSrv.response.success);
    EXPECT_EQ(initSrv.response.message, "initialized (armed)");

    can_driver::Init initAgainSrv;
    initAgainSrv.request.device = "fake0";
    initAgainSrv.request.loopback = false;
    ASSERT_TRUE(initClient.call(initAgainSrv));
    EXPECT_TRUE(initAgainSrv.response.success);
    EXPECT_EQ(initAgainSrv.response.message, "already initialized");

    std_srvs::Trigger resumeSrv;
    ASSERT_TRUE(resumeClient.call(resumeSrv));
    EXPECT_TRUE(resumeSrv.response.success);
    EXPECT_EQ(resumeSrv.response.message, "resumed");

    std_srvs::Trigger resumeAgainSrv;
    ASSERT_TRUE(resumeClient.call(resumeAgainSrv));
    EXPECT_TRUE(resumeAgainSrv.response.success);
    EXPECT_EQ(resumeAgainSrv.response.message, "already running");

    std_srvs::Trigger haltSrv;
    ASSERT_TRUE(haltClient.call(haltSrv));
    EXPECT_TRUE(haltSrv.response.success);
    EXPECT_EQ(haltSrv.response.message, "halted");

    std_srvs::Trigger haltAgainSrv;
    ASSERT_TRUE(haltClient.call(haltAgainSrv));
    EXPECT_TRUE(haltAgainSrv.response.success);
    EXPECT_EQ(haltAgainSrv.response.message, "already halted");

    std_srvs::Trigger disableSrv;
    ASSERT_TRUE(disableClient.call(disableSrv));
    EXPECT_TRUE(disableSrv.response.success);
    EXPECT_EQ(disableSrv.response.message, "disabled (standby)");

    std_srvs::Trigger enableSrv;
    ASSERT_TRUE(enableClient.call(enableSrv));
    EXPECT_TRUE(enableSrv.response.success);
    EXPECT_EQ(enableSrv.response.message, "enabled (armed)");

    std_srvs::Trigger enableAgainSrv;
    ASSERT_TRUE(enableClient.call(enableAgainSrv));
    EXPECT_TRUE(enableAgainSrv.response.success);
    EXPECT_EQ(enableAgainSrv.response.message, "already enabled");

    coordinator.SetFaulted();

    can_driver::Recover recoverSrv;
    recoverSrv.request.motor_id = 0xFFFFu;
    ASSERT_TRUE(recoverClient.call(recoverSrv));
    EXPECT_TRUE(recoverSrv.response.success);
    EXPECT_EQ(recoverSrv.response.message, "recovered (standby)");

    can_driver::Shutdown shutdownSrv;
    shutdownSrv.request.force = false;
    ASSERT_TRUE(shutdownClient.call(shutdownSrv));
    EXPECT_TRUE(shutdownSrv.response.success);
    EXPECT_EQ(shutdownSrv.response.message, "communication stopped; call ~/init first");

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, WriteAutoStopOnFaultAndBlockMotion)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_fault"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("safety_stop_on_fault", true);
    pnh.setParam("safety_require_enabled_for_motion", false);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_wheel/cmd_velocity");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);
    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 2.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    fakeDm->protocol()->setFault(true);
    hw.write(ros::Time::now(), ros::Duration(0.01));
    hw.write(ros::Time::now(), ros::Duration(0.01));

    EXPECT_EQ(fakeDm->protocol()->stopCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 0);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, WriteHoldAfterDeviceRecoverClearsStaleDirectCommand)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_recover_hold"));

    pnh.setParam("joints", makeSingleVelocityJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.2);
    pnh.setParam("safety_stop_on_fault", false);
    pnh.setParam("safety_require_enabled_for_motion", false);
    pnh.setParam("safety_hold_after_device_recover", true);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_wheel/cmd_velocity");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);
    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 1.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);

    fakeDm->setReady(false);
    msg.data = 3.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);

    fakeDm->setReady(true);
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);

    // 掉线期间积累的 direct 命令应被清空；恢复后需 fresh 命令才会再次下发。
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 1);

    msg.data = 4.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();
    hw.write(ros::Time::now(), ros::Duration(0.01));
    EXPECT_EQ(fakeDm->protocol()->velocityCalls(), 2);
    EXPECT_EQ(fakeDm->protocol()->lastVelocity(), 40);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, InitCspJointSetsModeAndPublishesRawFeedbackFromPprConfig)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    fakeDm->protocol()->setFeedbackPosition(16384);
    fakeDm->protocol()->setFeedbackVelocity(512);

    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_init"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));
    EXPECT_EQ(fakeDm->protocol()->setModeCalls(), 0);

    std::mutex stateMutex;
    can_driver::MotorState latestState;
    bool gotState = false;
    ros::Subscriber stateSub = nh.subscribe<can_driver::MotorState>(
        pnh.resolveName("motor_states"), 1,
        [&stateMutex, &latestState, &gotState](const can_driver::MotorState::ConstPtr &msg) {
            std::lock_guard<std::mutex> lock(stateMutex);
            latestState = *msg;
            gotState = true;
        });

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const auto initResult = hw.operationalCoordinator().RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;
    EXPECT_EQ(fakeDm->protocol()->setModeCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->lastModeMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastMode(), CanProtocol::MotorMode::CSP);

    for (int i = 0; i < 20 && !gotState; ++i) {
        ros::Duration(0.01).sleep();
    }

    ASSERT_TRUE(gotState);
    EXPECT_EQ(latestState.motor_id, 0x05u);
    EXPECT_EQ(latestState.position, 16384);
    EXPECT_EQ(latestState.velocity, 512);
    EXPECT_EQ(latestState.mode, can_driver::MotorState::MODE_CSP);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, RunningCspJointUsesQuickSetPositionWithPprScale)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>();
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_write"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("debug_bypass_ros_control", true);
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));
    enterRunning(hw);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    const std::string cmdTopic = pnh.resolveName("motor/test_arm/cmd_position");
    ros::Publisher pub = nh.advertise<std_msgs::Float64>(cmdTopic, 1);

    for (int i = 0; i < 20 && pub.getNumSubscribers() == 0; ++i) {
        ros::Duration(0.01).sleep();
    }

    std_msgs::Float64 msg;
    msg.data = 1.0;
    pub.publish(msg);
    ros::Duration(0.05).sleep();

    hw.write(ros::Time::now(), ros::Duration(0.01));

    const int32_t expectedRaw =
        static_cast<int32_t>(std::llround(1.0 / (2.0 * M_PI / 65536.0)));
    EXPECT_EQ(fakeDm->protocol()->quickPositionCalls(), 1);
    EXPECT_EQ(fakeDm->protocol()->positionCalls(), 0);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPositionMotor(), 0x05u);
    EXPECT_EQ(fakeDm->protocol()->lastQuickPosition(), expectedRaw);

    spinner.stop();
}

TEST_F(CanDriverHWSmokeTest, CspReleaseRequiresSharedStateModeMatchBeforeRunning)
{
    auto fakeDm = std::make_shared<FakeDeviceManager>(true);
    CanDriverHW hw(fakeDm);

    ros::NodeHandle nh;
    ros::NodeHandle pnh(uniqueNs("can_driver_hw_smoke_csp_lifecycle"));

    pnh.setParam("joints", makeSingleCspJoint());
    pnh.setParam("motor_state_period_sec", 0.05);

    ASSERT_TRUE(hw.init(nh, pnh));

    auto &coordinator = hw.operationalCoordinator();
    const auto initResult = coordinator.RequestInit("fake0", false);
    ASSERT_TRUE(initResult.ok) << initResult.message;
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Armed);

    const auto axisKey = can_driver::MakeAxisKey("fake0", CanType::PP, static_cast<MotorID>(0x05));
    fakeDm->sharedState()->mutateDeviceHealth(
        "fake0",
        [](can_driver::SharedDriverState::DeviceHealthState *health) {
            health->transportReady = true;
        });
    fakeDm->sharedState()->mutateAxisCommand(
        axisKey,
        [](can_driver::SharedDriverState::AxisCommandState *command) {
            command->desiredMode = CanProtocol::MotorMode::CSP;
            command->desiredModeValid = true;
            command->valid = false;
        });
    fakeDm->sharedState()->setAxisIntent(axisKey, can_driver::AxisIntent::Enable);
    fakeDm->sharedState()->mutateAxisFeedback(
        axisKey,
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->feedbackSeen = true;
            feedback->enabled = true;
            feedback->mode = CanProtocol::MotorMode::Position;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
        });

    const auto blockedRelease = coordinator.RequestRelease();
    EXPECT_FALSE(blockedRelease.ok);
    EXPECT_EQ(blockedRelease.message, "Mode not ready.");
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Armed);

    fakeDm->sharedState()->mutateAxisFeedback(
        axisKey,
        [](can_driver::SharedDriverState::AxisFeedbackState *feedback) {
            feedback->mode = CanProtocol::MotorMode::CSP;
            feedback->lastRxSteadyNs = can_driver::SharedDriverSteadyNowNs();
        });

    const auto releaseResult = coordinator.RequestRelease();
    EXPECT_TRUE(releaseResult.ok) << releaseResult.message;
    EXPECT_EQ(coordinator.mode(), can_driver::SystemOpMode::Running);
}

} // namespace
