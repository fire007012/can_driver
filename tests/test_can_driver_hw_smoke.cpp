#include <gtest/gtest.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>

#include "can_driver/CanDriverHW.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/lifecycle_service_gateway.hpp"
#include "can_driver/MotorCommand.h"
#include "can_driver/Recover.h"

#include <atomic>
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

    int64_t getPosition(MotorID) const override { return 0; }
    int16_t getCurrent(MotorID) const override { return 0; }
    int16_t getVelocity(MotorID) const override { return 0; }
    void initializeMotorRefresh(const std::vector<MotorID> &) override {}

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

    MotorID lastEnableMotor_{MotorID::LeftWheel};
    int enableCalls_{0};
    int stopCalls_{0};
    bool enabled_{true};
    bool hasFault_{false};
};

class FakeDeviceManager : public IDeviceManager {
public:
    FakeDeviceManager()
        : protocol_(std::make_shared<FakeProtocol>()), mutex_(std::make_shared<std::mutex>())
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

    std::shared_ptr<FakeProtocol> protocol() const { return protocol_; }
    void setReady(bool ready)
    {
        std::lock_guard<std::mutex> lock(readyMutex_);
        ready_ = ready;
    }

private:
    std::shared_ptr<FakeProtocol> protocol_;
    std::shared_ptr<std::mutex> mutex_;
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
    EXPECT_TRUE(srv.response.success);
    EXPECT_EQ(fakeDm->protocol()->enableCalls(), 1);
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

} // namespace
