#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/DeviceRefreshWorker.h"
#include "can_driver/DeviceManager.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <thread>

class DeviceManagerTestAccessor {
public:
    static void injectTransport(DeviceManager &dm,
                                const std::string &device,
                                std::shared_ptr<SocketCanController> transport)
    {
        dm.transports_[device] = std::move(transport);
        dm.deviceCmdMutexes_[device] = std::make_shared<std::mutex>();
    }

    static void seedTransportHealth(SocketCanController &transport,
                                    bool initialized,
                                    std::uint64_t txLinkUnavailable,
                                    std::int64_t lastTxLinkUnavailableSteadyNs,
                                    std::int64_t lastRxSteadyNs)
    {
        transport.initialized_.store(initialized);
        transport.txLinkUnavailableCount_.store(txLinkUnavailable);
        transport.lastTxLinkUnavailableSteadyNs_.store(lastTxLinkUnavailableSteadyNs);
        transport.lastRxSteadyNs_.store(lastRxSteadyNs);
    }

    static bool hasTransport(const DeviceManager &dm, const std::string &device)
    {
        return dm.transports_.find(device) != dm.transports_.end();
    }

    static std::size_t refreshWorkerCount(const DeviceManager &dm)
    {
        return dm.deviceRefreshRuntimes_.size();
    }

    static double effectiveRefreshRateHz(const DeviceManager &dm, const std::string &device)
    {
        return dm.effectiveRefreshRateHzLocked(device);
    }
};

class EyouCanTestAccessor {
public:
    static double refreshRateHz(const EyouCan &protocol)
    {
        return protocol.refreshRateHz_.load(std::memory_order_relaxed);
    }
};

namespace {

class RosTimeFixture : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }
};

bool hasVcan0()
{
    // 仅在系统已创建 vcan0 时运行真实 transport 初始化路径。
    return std::filesystem::exists("/sys/class/net/vcan0");
}

} // namespace

TEST_F(RosTimeFixture, EnsureTransportCreatesDevice)
{
    if (!hasVcan0()) {
        GTEST_SKIP() << "vcan0 not available; skipping transport initialization path.";
    }
    DeviceManager dm;
    // 使用 loopback 模式避免需要真实物理 CAN 设备。
    const bool ok = dm.ensureTransport("vcan0", true);
    ASSERT_TRUE(ok);
    EXPECT_NE(dm.getTransport("vcan0"), nullptr);
    EXPECT_EQ(dm.deviceCount(), 1u);
}

TEST_F(RosTimeFixture, EnsureTransportIdempotent)
{
    if (!hasVcan0()) {
        GTEST_SKIP() << "vcan0 not available; skipping transport idempotency path.";
    }
    DeviceManager dm;
    const bool ok1 = dm.ensureTransport("vcan0", true);
    const bool ok2 = dm.ensureTransport("vcan0", true);

    ASSERT_TRUE(ok1);
    EXPECT_TRUE(ok2);
    EXPECT_EQ(dm.deviceCount(), 1u);
}

TEST_F(RosTimeFixture, GetTransportReturnsNullForNonexistent)
{
    DeviceManager dm;
    EXPECT_EQ(dm.getTransport("nonexistent"), nullptr);
}

TEST_F(RosTimeFixture, GetProtocolReturnsNullBeforeEnsure)
{
    DeviceManager dm;
    EXPECT_EQ(dm.getProtocol("vcan0", CanType::MT), nullptr);
}

TEST_F(RosTimeFixture, SharedDriverStateIsStableAndAvailable)
{
    DeviceManager dm;

    const auto first = dm.getSharedDriverState();
    const auto second = dm.getSharedDriverState();

    ASSERT_NE(first, nullptr);
    EXPECT_EQ(first, second);
}

TEST_F(RosTimeFixture, DeviceRefreshWorkerRunsTickLoopUntilStopped)
{
    std::atomic<int> tickCount{0};
    can_driver::DeviceRefreshWorker worker(
        [&tickCount]() { ++tickCount; },
        []() { return std::chrono::milliseconds(1); });

    worker.start();

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(50);
    while (tickCount.load() < 2 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    worker.stop();
    EXPECT_GE(tickCount.load(), 2);
    EXPECT_FALSE(worker.running());
}

TEST_F(RosTimeFixture, EnsureProtocolCreatesProtocol)
{
    if (!hasVcan0()) {
        GTEST_SKIP() << "vcan0 not available; skipping protocol creation path.";
    }
    DeviceManager dm;
    const bool transportOk = dm.ensureTransport("vcan0", true);
    ASSERT_TRUE(transportOk);
    const bool protoOk = dm.ensureProtocol("vcan0", CanType::MT);
    EXPECT_TRUE(protoOk);
    EXPECT_NE(dm.getProtocol("vcan0", CanType::MT), nullptr);
}

TEST_F(RosTimeFixture, EnsureProtocolFailsWithoutTransport)
{
    DeviceManager dm;
    const bool ok = dm.ensureProtocol("nonexistent", CanType::MT);
    EXPECT_FALSE(ok);
}

TEST_F(RosTimeFixture, EnsureEcbProtocolCreatesWithoutSocketTransport)
{
    DeviceManager dm;
    ASSERT_TRUE(dm.ensureTransport("ecb://auto", true));
    ASSERT_TRUE(dm.ensureProtocol("ecb://auto", CanType::ECB));

    const auto protocol = dm.getProtocol("ecb://auto", CanType::ECB);
    ASSERT_NE(protocol, nullptr);
    EXPECT_TRUE(dm.isDeviceReady("ecb://auto"));
}

TEST_F(RosTimeFixture, GetDeviceMutexReturnsNullForNonexistent)
{
    DeviceManager dm;
    EXPECT_EQ(dm.getDeviceMutex("nonexistent"), nullptr);
}

TEST_F(RosTimeFixture, GetDeviceMutexReturnsValidAfterEnsure)
{
    if (!hasVcan0()) {
        GTEST_SKIP() << "vcan0 not available; skipping mutex creation path.";
    }
    DeviceManager dm;
    const bool ok = dm.ensureTransport("vcan0", true);
    ASSERT_TRUE(ok);
    auto mutex = dm.getDeviceMutex("vcan0");
    EXPECT_NE(mutex, nullptr);
}

TEST_F(RosTimeFixture, ShutdownAllClearsDevices)
{
    if (!hasVcan0()) {
        GTEST_SKIP() << "vcan0 not available; skipping shutdown-on-initialized-device path.";
    }
    DeviceManager dm;
    ASSERT_TRUE(dm.ensureTransport("vcan0", true));
    dm.shutdownAll();

    EXPECT_EQ(dm.deviceCount(), 0u);
    EXPECT_EQ(dm.getTransport("vcan0"), nullptr);
}

TEST_F(RosTimeFixture, ShutdownDeviceOnlyClearsRequestedDevice)
{
    DeviceManager dm;
    auto fake0 = std::make_shared<SocketCanController>();
    auto fake1 = std::make_shared<SocketCanController>();

    DeviceManagerTestAccessor::injectTransport(dm, "fake0", fake0);
    DeviceManagerTestAccessor::injectTransport(dm, "fake1", fake1);

    dm.getSharedDriverState()->mutateDeviceHealth(
        "fake0",
        [](can_driver::SharedDriverState::DeviceHealthState *health) {
            health->transportReady = true;
        });
    dm.getSharedDriverState()->mutateDeviceHealth(
        "fake1",
        [](can_driver::SharedDriverState::DeviceHealthState *health) {
            health->transportReady = true;
        });

    dm.shutdownDevice("fake0");

    EXPECT_EQ(dm.getTransport("fake0"), nullptr);
    EXPECT_EQ(dm.getDeviceMutex("fake0"), nullptr);
    EXPECT_EQ(dm.deviceCount(), 1u);
    EXPECT_TRUE(DeviceManagerTestAccessor::hasTransport(dm, "fake1"));
    EXPECT_NE(dm.getTransport("fake1"), nullptr);
    EXPECT_NE(dm.getDeviceMutex("fake1"), nullptr);

    can_driver::SharedDriverState::DeviceHealthState fake0Health;
    can_driver::SharedDriverState::DeviceHealthState fake1Health;
    ASSERT_TRUE(dm.getSharedDriverState()->getDeviceHealth("fake0", &fake0Health));
    ASSERT_TRUE(dm.getSharedDriverState()->getDeviceHealth("fake1", &fake1Health));
    EXPECT_FALSE(fake0Health.transportReady);
    EXPECT_TRUE(fake1Health.transportReady);
}

TEST_F(RosTimeFixture, InitDeviceRecreatesProtocolsAfterTransportReinit)
{
    if (!hasVcan0()) {
        GTEST_SKIP() << "vcan0 not available; skipping transport re-init regression path.";
    }

    DeviceManager dm;
    ASSERT_TRUE(dm.ensureTransport("vcan0", true));
    ASSERT_TRUE(dm.ensureProtocol("vcan0", CanType::MT));
    ASSERT_TRUE(dm.ensureProtocol("vcan0", CanType::PP));

    const auto oldMt = dm.getProtocol("vcan0", CanType::MT);
    const auto oldPp = dm.getProtocol("vcan0", CanType::PP);
    ASSERT_NE(oldMt, nullptr);
    ASSERT_NE(oldPp, nullptr);

    ASSERT_TRUE(dm.initDevice(
        "vcan0",
        {
            {CanType::MT, static_cast<MotorID>(0x01)},
            {CanType::PP, static_cast<MotorID>(0x02)},
        },
        true));

    const auto newMt = dm.getProtocol("vcan0", CanType::MT);
    const auto newPp = dm.getProtocol("vcan0", CanType::PP);
    ASSERT_NE(newMt, nullptr);
    ASSERT_NE(newPp, nullptr);
    EXPECT_NE(newMt, oldMt);
    EXPECT_NE(newPp, oldPp);
    EXPECT_EQ(DeviceManagerTestAccessor::refreshWorkerCount(dm), 1u);
}

TEST_F(RosTimeFixture, IsDeviceReadyDropsImmediatelyAfterRecentLinkDown)
{
    DeviceManager dm;
    auto transport = std::make_shared<SocketCanController>();
    DeviceManagerTestAccessor::injectTransport(dm, "fake0", transport);
    DeviceManagerTestAccessor::seedTransportHealth(
        *transport,
        true,
        1,
        200,
        100);

    EXPECT_FALSE(dm.isDeviceReady("fake0"));

    can_driver::SharedDriverState::DeviceHealthState health;
    ASSERT_TRUE(dm.getSharedDriverState()->getDeviceHealth("fake0", &health));
    EXPECT_FALSE(health.transportReady);
    EXPECT_EQ(health.txLinkUnavailable, 1u);
    EXPECT_EQ(health.lastTxLinkUnavailableSteadyNs, 200);
    EXPECT_EQ(health.lastRxSteadyNs, 100);
}

TEST_F(RosTimeFixture, IsDeviceReadyRecoversAfterRxNewerThanLinkDown)
{
    DeviceManager dm;
    auto transport = std::make_shared<SocketCanController>();
    DeviceManagerTestAccessor::injectTransport(dm, "fake0", transport);
    DeviceManagerTestAccessor::seedTransportHealth(
        *transport,
        true,
        1,
        100,
        200);

    EXPECT_TRUE(dm.isDeviceReady("fake0"));

    can_driver::SharedDriverState::DeviceHealthState health;
    ASSERT_TRUE(dm.getSharedDriverState()->getDeviceHealth("fake0", &health));
    EXPECT_TRUE(health.transportReady);
    EXPECT_EQ(health.txLinkUnavailable, 1u);
    EXPECT_EQ(health.lastTxLinkUnavailableSteadyNs, 100);
    EXPECT_EQ(health.lastRxSteadyNs, 200);
}

TEST_F(RosTimeFixture, DeviceRefreshOverrideDoesNotLeakAcrossDevices)
{
    DeviceManager dm;
    auto fake0 = std::make_shared<SocketCanController>();
    auto fake1 = std::make_shared<SocketCanController>();

    DeviceManagerTestAccessor::injectTransport(dm, "fake0", fake0);
    DeviceManagerTestAccessor::injectTransport(dm, "fake1", fake1);

    dm.setRefreshRateHz(20.0);
    dm.setDeviceRefreshRateHz("fake0", 5.0);

    ASSERT_TRUE(dm.ensureProtocol("fake0", CanType::PP));
    ASSERT_TRUE(dm.ensureProtocol("fake1", CanType::PP));

    auto fake0Protocol = std::dynamic_pointer_cast<EyouCan>(dm.getProtocol("fake0", CanType::PP));
    auto fake1Protocol = std::dynamic_pointer_cast<EyouCan>(dm.getProtocol("fake1", CanType::PP));
    ASSERT_NE(fake0Protocol, nullptr);
    ASSERT_NE(fake1Protocol, nullptr);
    EXPECT_DOUBLE_EQ(EyouCanTestAccessor::refreshRateHz(*fake0Protocol), 5.0);
    EXPECT_DOUBLE_EQ(EyouCanTestAccessor::refreshRateHz(*fake1Protocol), 20.0);

    dm.setRefreshRateHz(50.0);
    EXPECT_DOUBLE_EQ(DeviceManagerTestAccessor::effectiveRefreshRateHz(dm, "fake0"), 5.0);
    EXPECT_DOUBLE_EQ(DeviceManagerTestAccessor::effectiveRefreshRateHz(dm, "fake1"), 50.0);
    EXPECT_DOUBLE_EQ(EyouCanTestAccessor::refreshRateHz(*fake0Protocol), 5.0);
    EXPECT_DOUBLE_EQ(EyouCanTestAccessor::refreshRateHz(*fake1Protocol), 50.0);

    dm.setDeviceRefreshRateHz("fake0", 0.0);
    EXPECT_DOUBLE_EQ(DeviceManagerTestAccessor::effectiveRefreshRateHz(dm, "fake0"), 50.0);
    EXPECT_DOUBLE_EQ(EyouCanTestAccessor::refreshRateHz(*fake0Protocol), 50.0);
}
