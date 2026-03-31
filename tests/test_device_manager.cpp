#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/DeviceManager.h"
#include "can_driver/ProtocolRefreshWorker.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <thread>

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

TEST_F(RosTimeFixture, ProtocolRefreshWorkerRunsTickLoopUntilStopped)
{
    std::atomic<int> tickCount{0};
    can_driver::ProtocolRefreshWorker worker(
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
