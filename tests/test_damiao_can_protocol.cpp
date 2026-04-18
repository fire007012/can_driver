#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/DamiaoCan.h"
#include "can_driver/SharedDriverState.h"

#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <mutex>
#include <vector>

namespace {

class MockTransport : public CanTransport {
public:
    SendResult send(const Frame &frame) override
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            sentFrames_.push_back(frame);
        }
        cv_.notify_all();
        return SendResult::Ok;
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

    void simulateReceive(const Frame &frame)
    {
        ReceiveHandler handler;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            handler = receiveHandler_;
        }
        if (handler) {
            handler(frame);
        }
    }

    std::vector<Frame> snapshotSentFrames() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return sentFrames_;
    }

    void clearSentFrames()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        sentFrames_.clear();
    }

    bool waitForSentFrameCount(std::size_t count, std::chrono::milliseconds timeout)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        return cv_.wait_for(
            lock, timeout, [this, count]() { return sentFrames_.size() >= count; });
    }

private:
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::vector<Frame> sentFrames_;
    ReceiveHandler receiveHandler_;
};

class MockTxDispatcher : public CanTxDispatcher {
public:
    explicit MockTxDispatcher(std::shared_ptr<MockTransport> transport)
        : transport_(std::move(transport))
    {
    }

    void submit(const Request &request) override
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            requests_.push_back(request);
        }
        if (autoSend_ && transport_) {
            const auto eventTime = std::chrono::steady_clock::now();
            const auto result = transport_->send(request.frame);
            if (request.completion) {
                request.completion(true, result, eventTime);
            }
        }
    }

    std::vector<Request> snapshotRequests() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return requests_;
    }

    void clearRequests()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.clear();
    }

private:
    std::shared_ptr<MockTransport> transport_;
    mutable std::mutex mutex_;
    std::vector<Request> requests_;
    bool autoSend_{true};
};

float decodeFloatLE(const CanTransport::Frame &frame)
{
    std::uint32_t raw = 0;
    raw |= static_cast<std::uint32_t>(frame.data[0]);
    raw |= static_cast<std::uint32_t>(frame.data[1]) << 8;
    raw |= static_cast<std::uint32_t>(frame.data[2]) << 16;
    raw |= static_cast<std::uint32_t>(frame.data[3]) << 24;

    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(value));
    return value;
}

CanTransport::Frame makeRegisterAck(std::uint8_t motorId,
                                    std::uint8_t registerId,
                                    std::uint32_t value)
{
    CanTransport::Frame frame {};
    frame.id = 0x000u;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 8;
    frame.data.fill(0);
    frame.data[0] = motorId;
    frame.data[1] = 0;
    frame.data[2] = 0x55u;
    frame.data[3] = registerId;
    frame.data[4] = static_cast<std::uint8_t>(value & 0xFFu);
    frame.data[5] = static_cast<std::uint8_t>((value >> 8) & 0xFFu);
    frame.data[6] = static_cast<std::uint8_t>((value >> 16) & 0xFFu);
    frame.data[7] = static_cast<std::uint8_t>((value >> 24) & 0xFFu);
    return frame;
}

CanTransport::Frame makeFeedbackFrame(std::uint8_t motorId,
                                      std::uint8_t state,
                                      std::uint16_t positionRaw = 0x8000u,
                                      std::uint16_t velocityRaw = 0x0800u,
                                      std::uint16_t torqueRaw = 0x0800u)
{
    CanTransport::Frame frame {};
    frame.id = 0x000u;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 8;
    frame.data.fill(0);
    frame.data[0] = static_cast<std::uint8_t>((state << 4) | (motorId & 0x0Fu));
    frame.data[1] = static_cast<std::uint8_t>((positionRaw >> 8) & 0xFFu);
    frame.data[2] = static_cast<std::uint8_t>(positionRaw & 0xFFu);
    frame.data[3] = static_cast<std::uint8_t>((velocityRaw >> 4) & 0xFFu);
    frame.data[4] = static_cast<std::uint8_t>(((velocityRaw & 0x0Fu) << 4) |
                                              ((torqueRaw >> 8) & 0x0Fu));
    frame.data[5] = static_cast<std::uint8_t>(torqueRaw & 0xFFu);
    return frame;
}

class DamiaoCanTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }

    DamiaoCanTest()
        : transport_(std::make_shared<MockTransport>())
        , txDispatcher_(std::make_shared<MockTxDispatcher>(transport_))
        , sharedState_(std::make_shared<can_driver::SharedDriverState>())
        , damiao_(transport_, txDispatcher_, sharedState_, "can0")
    {
    }

    void configureVelocityMode(MotorID motorId)
    {
        const auto nodeId = can_driver::toProtocolNodeId(motorId);
        auto future = std::async(std::launch::async, [&]() {
            return damiao_.setMode(motorId, CanProtocol::MotorMode::Velocity);
        });

        ASSERT_TRUE(transport_->waitForSentFrameCount(1, std::chrono::milliseconds(200)));
        const auto frames = transport_->snapshotSentFrames();
        ASSERT_EQ(frames.size(), 1u);
        transport_->simulateReceive(makeRegisterAck(nodeId, 10u, 3u));
        ASSERT_TRUE(future.get());

        transport_->clearSentFrames();
        txDispatcher_->clearRequests();
    }

    const can_driver::SharedDriverState::AxisKey axisKey(MotorID motorId) const
    {
        return can_driver::MakeAxisKey("can0", CanType::DM, motorId);
    }

    std::shared_ptr<MockTransport> transport_;
    std::shared_ptr<MockTxDispatcher> txDispatcher_;
    std::shared_ptr<can_driver::SharedDriverState> sharedState_;
    DamiaoCan damiao_;
};

} // namespace

TEST_F(DamiaoCanTest, SetModeWritesLegacyVelocityRegisterFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);

    auto future = std::async(std::launch::async, [&]() {
        return damiao_.setMode(kMotorId, CanProtocol::MotorMode::Velocity);
    });

    ASSERT_TRUE(transport_->waitForSentFrameCount(1, std::chrono::milliseconds(200)));
    const auto frames = transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(), 1u);

    const auto &frame = frames[0];
    EXPECT_EQ(frame.id, 0x7FFu);
    EXPECT_FALSE(frame.isExtended);
    EXPECT_FALSE(frame.isRemoteRequest);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0x01u);
    EXPECT_EQ(frame.data[1], 0x00u);
    EXPECT_EQ(frame.data[2], 0x55u);
    EXPECT_EQ(frame.data[3], 10u);
    EXPECT_EQ(frame.data[4], 0x03u);
    EXPECT_EQ(frame.data[5], 0x00u);
    EXPECT_EQ(frame.data[6], 0x00u);
    EXPECT_EQ(frame.data[7], 0x00u);

    transport_->simulateReceive(makeRegisterAck(0x01u, 10u, 3u));
    EXPECT_TRUE(future.get());

    can_driver::SharedDriverState::AxisCommandState command;
    ASSERT_TRUE(sharedState_->getAxisCommand(axisKey(kMotorId), &command));
    EXPECT_EQ(command.desiredMode, CanProtocol::MotorMode::Velocity);
    EXPECT_TRUE(command.desiredModeValid);
    EXPECT_FALSE(command.valid);
}

TEST_F(DamiaoCanTest, SetVelocityEncodesLegacySpeedFrameAndSharedCommand)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    configureVelocityMode(kMotorId);

    ASSERT_TRUE(damiao_.setVelocity(kMotorId, 12345));

    const auto frames = transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].id, 0x201u);
    EXPECT_EQ(frames[0].dlc, 4u);
    EXPECT_FLOAT_EQ(decodeFloatLE(frames[0]), 1.2345f);

    const auto requests = txDispatcher_->snapshotRequests();
    ASSERT_EQ(requests.size(), 1u);
    EXPECT_EQ(requests[0].category, CanTxDispatcher::Category::Control);
    EXPECT_STREQ(requests[0].source, "DamiaoCan::setVelocity");

    can_driver::SharedDriverState::AxisCommandState command;
    ASSERT_TRUE(sharedState_->getAxisCommand(axisKey(kMotorId), &command));
    EXPECT_EQ(command.targetVelocity, 12345);
    EXPECT_EQ(command.desiredMode, CanProtocol::MotorMode::Velocity);
    EXPECT_TRUE(command.desiredModeValid);
    EXPECT_TRUE(command.valid);
    EXPECT_GT(command.lastCommandSteadyNs, 0);
    EXPECT_EQ(sharedState_->getAxisIntent(axisKey(kMotorId)), can_driver::AxisIntent::Run);
}

TEST_F(DamiaoCanTest, EnableSendsLegacyEnableThenNeutralSequence)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    configureVelocityMode(kMotorId);

    auto future = std::async(std::launch::async, [&]() { return damiao_.Enable(kMotorId); });

    ASSERT_TRUE(transport_->waitForSentFrameCount(2, std::chrono::milliseconds(300)));
    const auto frames = transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(), 2u);

    EXPECT_EQ(frames[0].id, 0x201u);
    EXPECT_EQ(frames[0].dlc, 8u);
    for (std::size_t i = 0; i < 7; ++i) {
        EXPECT_EQ(frames[0].data[i], 0xFFu);
    }
    EXPECT_EQ(frames[0].data[7], 0xFCu);

    EXPECT_EQ(frames[1].id, 0x201u);
    EXPECT_EQ(frames[1].dlc, 4u);
    EXPECT_FLOAT_EQ(decodeFloatLE(frames[1]), 0.0f);

    transport_->simulateReceive(makeFeedbackFrame(0x01u, 0x01u));
    EXPECT_TRUE(future.get());
    EXPECT_EQ(sharedState_->getAxisIntent(axisKey(kMotorId)), can_driver::AxisIntent::Enable);
}

TEST_F(DamiaoCanTest, DisableSendsNeutralThenLegacyDisableSequence)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    configureVelocityMode(kMotorId);
    ASSERT_TRUE(damiao_.setVelocity(kMotorId, 5000));
    transport_->clearSentFrames();
    txDispatcher_->clearRequests();

    auto future = std::async(std::launch::async, [&]() { return damiao_.Disable(kMotorId); });

    ASSERT_TRUE(transport_->waitForSentFrameCount(2, std::chrono::milliseconds(200)));
    const auto frames = transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(), 2u);

    EXPECT_EQ(frames[0].id, 0x201u);
    EXPECT_EQ(frames[0].dlc, 4u);
    EXPECT_FLOAT_EQ(decodeFloatLE(frames[0]), 0.0f);

    EXPECT_EQ(frames[1].id, 0x201u);
    EXPECT_EQ(frames[1].dlc, 8u);
    EXPECT_EQ(frames[1].data[7], 0xFDu);

    transport_->simulateReceive(makeFeedbackFrame(0x01u, 0x00u));
    EXPECT_TRUE(future.get());
}

TEST_F(DamiaoCanTest, FeedbackFrameUpdatesSharedStateUsingLegacyBitPacking)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    configureVelocityMode(kMotorId);

    transport_->simulateReceive(makeFeedbackFrame(0x01u, 0x01u));

    EXPECT_TRUE(damiao_.isEnabled(kMotorId));
    EXPECT_FALSE(damiao_.hasFault(kMotorId));
    EXPECT_EQ(damiao_.getPosition(kMotorId), 2);
    EXPECT_EQ(damiao_.getVelocity(kMotorId), 110);
    EXPECT_EQ(damiao_.getCurrent(kMotorId), 0);

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    ASSERT_TRUE(sharedState_->getAxisFeedback(axisKey(kMotorId), &feedback));
    EXPECT_TRUE(feedback.feedbackSeen);
    EXPECT_TRUE(feedback.positionValid);
    EXPECT_TRUE(feedback.velocityValid);
    EXPECT_TRUE(feedback.currentValid);
    EXPECT_TRUE(feedback.modeValid);
    EXPECT_EQ(feedback.mode, CanProtocol::MotorMode::Velocity);
    EXPECT_TRUE(feedback.enabledValid);
    EXPECT_TRUE(feedback.enabled);
    EXPECT_TRUE(feedback.faultValid);
    EXPECT_FALSE(feedback.fault);
    EXPECT_GT(feedback.lastRxSteadyNs, 0);
}

TEST_F(DamiaoCanTest, KeepaliveRefreshReusesLastVelocityCommand)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    configureVelocityMode(kMotorId);
    ASSERT_TRUE(damiao_.setVelocity(kMotorId, -2500));
    transport_->clearSentFrames();
    txDispatcher_->clearRequests();

    ASSERT_TRUE(damiao_.issueRefreshQuery(kMotorId, DamiaoCan::RefreshQuery::Keepalive));

    const auto frames = transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].id, 0x201u);
    EXPECT_EQ(frames[0].dlc, 4u);
    EXPECT_FLOAT_EQ(decodeFloatLE(frames[0]), -0.25f);

    const auto requests = txDispatcher_->snapshotRequests();
    ASSERT_EQ(requests.size(), 1u);
    EXPECT_EQ(requests[0].category, CanTxDispatcher::Category::Query);
    EXPECT_STREQ(requests[0].source, "DamiaoCan::issueRefreshQuery");
}
