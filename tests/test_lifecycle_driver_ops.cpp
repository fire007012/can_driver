#include <gtest/gtest.h>

#include "can_driver/IDeviceManager.h"
#include "can_driver/lifecycle_driver_ops.hpp"

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace {

class FakeProtocol : public CanProtocol {
public:
    bool setMode(MotorID, MotorMode) override { return true; }
    bool setVelocity(MotorID, int32_t) override { return true; }
    bool setAcceleration(MotorID, int32_t) override { return true; }
    bool setDeceleration(MotorID, int32_t) override { return true; }
    bool setPosition(MotorID, int32_t) override { return true; }

    bool Enable(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto id = static_cast<uint16_t>(motorId);
        ++enableCalls_[id];
        const bool result = lookupOr(enableResult_, id, true);
        if (result) {
            enabled_[id] = true;
        }
        return result;
    }

    bool Disable(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto id = static_cast<uint16_t>(motorId);
        ++disableCalls_[id];
        const bool result = lookupOr(disableResult_, id, true);
        if (!result) {
            return false;
        }
        enabled_[id] = false;
        return true;
    }

    bool Stop(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stopCalls_[static_cast<uint16_t>(motorId)];
        return true;
    }

    bool ResetFault(MotorID motorId) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto id = static_cast<uint16_t>(motorId);
        ++resetFaultCalls_[id];
        const bool result = lookupOr(resetFaultResult_, id, true);
        if (result && lookupOr(clearFaultOnReset_, id, true)) {
            fault_[id] = false;
        }
        return result;
    }

    int64_t getPosition(MotorID) const override { return 0; }
    int16_t getCurrent(MotorID) const override { return 0; }
    int16_t getVelocity(MotorID) const override { return 0; }
    void initializeMotorRefresh(const std::vector<MotorID> &) override {}

    bool isEnabled(MotorID motorId) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lookupOr(enabled_, static_cast<uint16_t>(motorId), false);
    }

    bool hasFault(MotorID motorId) const override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lookupOr(fault_, static_cast<uint16_t>(motorId), false);
    }

    void setEnableResult(uint16_t motorId, bool result)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        enableResult_[motorId] = result;
    }

    void setResetFaultBehavior(uint16_t motorId, bool result, bool clearFault)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        resetFaultResult_[motorId] = result;
        clearFaultOnReset_[motorId] = clearFault;
    }

    void setFault(uint16_t motorId, bool value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        fault_[motorId] = value;
    }

    void setDisableResult(uint16_t motorId, bool result)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        disableResult_[motorId] = result;
    }

    int disableCalls(uint16_t motorId) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lookupOr(disableCalls_, motorId, 0);
    }

    int resetFaultCalls(uint16_t motorId) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return lookupOr(resetFaultCalls_, motorId, 0);
    }

private:
    template <typename T>
    static T lookupOr(const std::map<uint16_t, T> &values, uint16_t key, T fallback)
    {
        const auto it = values.find(key);
        return it == values.end() ? fallback : it->second;
    }

    mutable std::mutex mutex_;
    std::map<uint16_t, bool> enableResult_;
    std::map<uint16_t, bool> disableResult_;
    std::map<uint16_t, bool> resetFaultResult_;
    std::map<uint16_t, bool> clearFaultOnReset_;
    std::map<uint16_t, bool> enabled_;
    std::map<uint16_t, bool> fault_;
    std::map<uint16_t, int> enableCalls_;
    std::map<uint16_t, int> disableCalls_;
    std::map<uint16_t, int> stopCalls_;
    std::map<uint16_t, int> resetFaultCalls_;
};

class FakeDeviceManager : public IDeviceManager {
public:
    FakeDeviceManager()
        : protocol_(std::make_shared<FakeProtocol>()),
          mutex_(std::make_shared<std::mutex>())
    {
    }

    bool ensureTransport(const std::string &, bool = false) override { return true; }
    bool ensureProtocol(const std::string &, CanType) override { return true; }

    bool initDevice(const std::string &device,
                    const std::vector<std::pair<CanType, MotorID>> &motors,
                    bool loopback = false) override
    {
        lastInitDevice_ = device;
        lastInitMotors_ = motors;
        lastInitLoopback_ = loopback;
        return initResult_;
    }

    void startRefresh(const std::string &, CanType, const std::vector<MotorID> &) override {}
    void setRefreshRateHz(double) override {}
    void setPpFastWriteEnabled(bool) override {}
    void shutdownAll() override { ++shutdownCalls_; }

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
        return ready_;
    }

    std::size_t deviceCount() const override { return 1; }

    std::shared_ptr<FakeProtocol> protocol() const { return protocol_; }

    void setReady(bool ready) { ready_ = ready; }
    void setInitResult(bool result) { initResult_ = result; }

    const std::string &lastInitDevice() const { return lastInitDevice_; }
    const std::vector<std::pair<CanType, MotorID>> &lastInitMotors() const { return lastInitMotors_; }
    bool lastInitLoopback() const { return lastInitLoopback_; }

private:
    std::shared_ptr<FakeProtocol> protocol_;
    std::shared_ptr<std::mutex> mutex_;
    bool ready_{true};
    bool initResult_{true};
    int shutdownCalls_{0};
    std::string lastInitDevice_;
    std::vector<std::pair<CanType, MotorID>> lastInitMotors_;
    bool lastInitLoopback_{false};
};

std::vector<MotorActionExecutor::Target> makeTargets()
{
    return {
        MotorActionExecutor::Target{"joint_a", "fake0", CanType::MT, static_cast<MotorID>(0x141)},
        MotorActionExecutor::Target{"joint_b", "fake0", CanType::MT, static_cast<MotorID>(0x142)},
        MotorActionExecutor::Target{"joint_c", "fake1", CanType::PP, static_cast<MotorID>(0x201)},
    };
}

TEST(LifecycleDriverOpsTest, EnableAllRollsBackSucceededMotorsOnPartialFailure)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets(makeTargets());

    deviceManager->protocol()->setEnableResult(0x141, true);
    deviceManager->protocol()->setEnableResult(0x142, false);

    const auto result = ops.enableAll();
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message, "Enable command rejected.");
    EXPECT_EQ(deviceManager->protocol()->disableCalls(0x141), 1);
    EXPECT_FALSE(deviceManager->protocol()->isEnabled(static_cast<MotorID>(0x141)));
}

TEST(LifecycleDriverOpsTest, EnableAllReportsRollbackFailureWhenDisableFails)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets(makeTargets());

    deviceManager->protocol()->setEnableResult(0x141, true);
    deviceManager->protocol()->setEnableResult(0x142, false);
    deviceManager->protocol()->setDisableResult(0x141, false);

    const auto result = ops.enableAll();
    EXPECT_FALSE(result.ok);
    EXPECT_EQ(result.message, "Enable rollback failed after partial success.");
    EXPECT_EQ(deviceManager->protocol()->disableCalls(0x141), 1);
    EXPECT_TRUE(deviceManager->protocol()->isEnabled(static_cast<MotorID>(0x141)));
}

TEST(LifecycleDriverOpsTest, InitializeDeviceFiltersTargetsByRequestedBus)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets(makeTargets());

    const auto result = ops.initializeDevice("fake1", true);
    ASSERT_TRUE(result.ok);
    EXPECT_EQ(deviceManager->lastInitDevice(), "fake1");
    ASSERT_EQ(deviceManager->lastInitMotors().size(), 1u);
    EXPECT_EQ(deviceManager->lastInitMotors().front().first, CanType::PP);
    EXPECT_EQ(static_cast<uint16_t>(deviceManager->lastInitMotors().front().second), 0x201u);
    EXPECT_TRUE(deviceManager->lastInitLoopback());
}

TEST(LifecycleDriverOpsTest, RecoverAllResetsFaultsBeforeReturningSuccess)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_a", "fake0", CanType::MT, static_cast<MotorID>(0x141)},
        MotorActionExecutor::Target{"joint_b", "fake0", CanType::MT, static_cast<MotorID>(0x142)},
    });

    deviceManager->protocol()->setFault(0x141, true);
    deviceManager->protocol()->setFault(0x142, true);
    deviceManager->protocol()->setResetFaultBehavior(0x141, true, true);
    deviceManager->protocol()->setResetFaultBehavior(0x142, true, true);

    const auto result = ops.recoverAll();
    EXPECT_TRUE(result.ok);
    EXPECT_EQ(result.message, "Recovered (standby).");
    EXPECT_EQ(deviceManager->protocol()->resetFaultCalls(0x141), 1);
    EXPECT_EQ(deviceManager->protocol()->resetFaultCalls(0x142), 1);
    EXPECT_FALSE(ops.anyFaultActive());
}

TEST(LifecycleDriverOpsTest, MotionHealthyRequiresReadyDeviceAndClearedFaults)
{
    auto deviceManager = std::make_shared<FakeDeviceManager>();
    MotorActionExecutor executor(deviceManager);
    can_driver::LifecycleDriverOps ops(deviceManager, &executor);
    ops.setTargets({
        MotorActionExecutor::Target{"joint_a", "fake0", CanType::MT, static_cast<MotorID>(0x141)},
    });

    std::string detail;
    deviceManager->setReady(false);
    EXPECT_FALSE(ops.motionHealthy(&detail));
    EXPECT_EQ(detail, "CAN device not ready.");

    deviceManager->setReady(true);
    deviceManager->protocol()->setFault(0x141, true);
    EXPECT_FALSE(ops.motionHealthy(&detail));
    EXPECT_EQ(detail, "Fault still active.");

    deviceManager->protocol()->setFault(0x141, false);
    detail.clear();
    EXPECT_TRUE(ops.motionHealthy(&detail));
    EXPECT_TRUE(detail.empty());
}

} // namespace
