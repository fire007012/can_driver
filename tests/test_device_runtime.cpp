#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/DeviceRuntime.h"

#include <chrono>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

namespace {

class MockTransport : public CanTransport {
public:
    void send(const Frame &frame) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sentFrames_.push_back(frame);
        senderThreads_.insert(std::this_thread::get_id());
    }

    std::size_t addReceiveHandler(ReceiveHandler handler) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        receiveHandler_ = std::move(handler);
        return 1;
    }

    void removeReceiveHandler(std::size_t) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        receiveHandler_ = nullptr;
    }

    std::vector<Frame> snapshotFrames() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sentFrames_;
    }

    std::size_t senderThreadCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return senderThreads_.size();
    }

private:
    mutable std::mutex mutex_;
    std::vector<Frame> sentFrames_;
    std::set<std::thread::id> senderThreads_;
    ReceiveHandler receiveHandler_;
};

CanTransport::Frame makeFrame(std::uint32_t id)
{
    CanTransport::Frame frame;
    frame.id = id;
    frame.dlc = 1;
    frame.data[0] = static_cast<std::uint8_t>(id & 0xFF);
    return frame;
}

class DeviceRuntimeTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }
};

} // namespace

TEST_F(DeviceRuntimeTest, DrainsQueuedFramesInPriorityOrder)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    DeviceRuntime runtime(transport, "test_can0", options);

    runtime.submit({makeFrame(0x04), CanTxDispatcher::Category::Query, "query"});
    runtime.submit({makeFrame(0x03), CanTxDispatcher::Category::Config, "config"});
    runtime.submit({makeFrame(0x02), CanTxDispatcher::Category::Recover, "recover"});
    runtime.submit({makeFrame(0x01), CanTxDispatcher::Category::Control, "control"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto frames = transport->snapshotFrames();
    ASSERT_EQ(frames.size(), 4u);
    EXPECT_EQ(frames[0].id, 0x01u);
    EXPECT_EQ(frames[1].id, 0x02u);
    EXPECT_EQ(frames[2].id, 0x03u);
    EXPECT_EQ(frames[3].id, 0x04u);
}

TEST_F(DeviceRuntimeTest, DropsQueryFramesWhenQueryQueueIsFull)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime::Options options;
    options.autostart = false;
    options.maxQueryQueueDepth = 1;
    DeviceRuntime runtime(transport, "test_can0", options);

    runtime.submit({makeFrame(0x11), CanTxDispatcher::Category::Query, "query1"});
    runtime.submit({makeFrame(0x12), CanTxDispatcher::Category::Query, "query2"});
    runtime.submit({makeFrame(0x10), CanTxDispatcher::Category::Control, "control"});

    runtime.start();
    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(200)));

    const auto frames = transport->snapshotFrames();
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[0].id, 0x10u);
    EXPECT_EQ(frames[1].id, 0x11u);

    const auto stats = runtime.snapshotStats();
    EXPECT_EQ(stats.droppedQuery, 1u);
}

TEST_F(DeviceRuntimeTest, UsesSingleWorkerThreadForAllTransmissions)
{
    auto transport = std::make_shared<MockTransport>();
    DeviceRuntime runtime(transport, "test_can0");

    std::vector<std::thread> producers;
    for (std::uint32_t producer = 0; producer < 4; ++producer) {
        producers.emplace_back([&runtime, producer]() {
            for (std::uint32_t i = 0; i < 8; ++i) {
                runtime.submit({makeFrame(0x100 + producer * 8 + i),
                                CanTxDispatcher::Category::Control,
                                "producer"});
            }
        });
    }

    for (auto &producer : producers) {
        producer.join();
    }

    ASSERT_TRUE(runtime.waitUntilIdleFor(std::chrono::milliseconds(500)));

    const auto frames = transport->snapshotFrames();
    EXPECT_EQ(frames.size(), 32u);
    EXPECT_EQ(transport->senderThreadCount(), 1u);
}
