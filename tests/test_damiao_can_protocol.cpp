#include <gtest/gtest.h>

#include "can_driver/DamiaoCan.h"
#include "can_driver/DirectCommandTimeout.h"
#include "can_driver/SharedDriverState.h"

#include <cmath>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <future>
#include <memory>
#include <limits>
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
        return sendResult;
    }

    SendResult sendResult{SendResult::Ok};

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

std::uint32_t encodeFloatBits(float value)
{
    std::uint32_t bits = 0;
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

std::uint32_t decodeUInt32LE(const CanTransport::Frame &frame, std::size_t index)
{
    if (index + 3 >= frame.dlc) {
        return 0;
    }
    return static_cast<std::uint32_t>(frame.data[index]) |
           (static_cast<std::uint32_t>(frame.data[index + 1]) << 8) |
           (static_cast<std::uint32_t>(frame.data[index + 2]) << 16) |
           (static_cast<std::uint32_t>(frame.data[index + 3]) << 24);
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
    }

    DamiaoCanTest()
        : transport_(std::make_shared<MockTransport>())
        , txDispatcher_(std::make_shared<MockTxDispatcher>(transport_))
        , sharedState_(std::make_shared<can_driver::SharedDriverState>())
        , damiao_(transport_, txDispatcher_, sharedState_, "canable0")
    {
    }

    void configureVelocityMode(MotorID motorId)
    {
        const auto nodeId = can_driver::toProtocolNodeId(motorId);
        auto future = std::async(std::launch::async, [&]() {
            return damiao_.setMode(motorId, CanProtocol::MotorMode::Velocity);
        });

        ackVelocityModeAndFeedbackRanges(nodeId);
        ASSERT_TRUE(future.get());

        transport_->clearSentFrames();
        txDispatcher_->clearRequests();
    }

    void ackVelocityModeAndFeedbackRanges(std::uint8_t nodeId, float p = 12.5f,
                                          float v = 45.0f, float t = 18.0f,
                                          std::uint32_t master = 0, std::uint32_t mode = 3)
    {
        std::size_t offset=0;
        if (mode == 1) {
            ASSERT_TRUE(transport_->waitForSentFrameCount(1, std::chrono::milliseconds(500)));
            const auto frame=transport_->snapshotSentFrames()[0];
            EXPECT_EQ(frame.data[2],0x33);EXPECT_EQ(frame.data[3],0x50);
            auto ack=makeRegisterAck(nodeId,0x50,encodeFloatBits(1.f));
            ack.id=master;ack.data[2]=0x33;transport_->simulateReceive(ack);offset=1;
        }
        const std::array<std::uint8_t, 5> registers{{10, 10, 21, 22, 23}};
        const std::array<std::uint32_t, 5> values{{mode, mode, encodeFloatBits(p), encodeFloatBits(v), encodeFloatBits(t)}};
        for (std::size_t i = 0; i < registers.size(); ++i) {
            ASSERT_TRUE(transport_->waitForSentFrameCount(i + 1 + offset, std::chrono::milliseconds(500)));
            const auto frames = transport_->snapshotSentFrames();
            ASSERT_EQ(frames.size(), i + 1 + offset);
            EXPECT_EQ(frames[i+offset].id, 0x7FFu);
            EXPECT_EQ(frames[i+offset].data[0], nodeId);
            EXPECT_EQ(frames[i+offset].data[2], i == 0 ? 0x55 : 0x33);
            EXPECT_EQ(frames[i+offset].data[3], registers[i]);
            auto ack = makeRegisterAck(nodeId, registers[i], values[i]);
            ack.id = master;
            ack.data[2] = i == 0 ? 0x55 : 0x33;
            transport_->simulateReceive(ack);
        }
    }

    const can_driver::SharedDriverState::AxisKey axisKey(MotorID motorId) const
    {
        return can_driver::MakeAxisKey("canable0", CanType::DM, motorId);
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
    constexpr std::uint8_t kNodeId = 0x01u;

    auto future = std::async(std::launch::async, [&]() {
        return damiao_.setMode(kMotorId, CanProtocol::MotorMode::Velocity);
    });

    ackVelocityModeAndFeedbackRanges(kNodeId);
    EXPECT_TRUE(future.get());

    const auto frames = transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(), 6u);

    // Startup must solicit a real state while disabled; never enable to get feedback.
    EXPECT_EQ(frames[5].id, 0x201u);
    EXPECT_EQ(frames[5].dlc, 4u);
    EXPECT_FLOAT_EQ(decodeFloatLE(frames[5]), 0.f);

    const auto &modeFrame = frames[0];
    EXPECT_EQ(modeFrame.id, 0x7FFu);
    EXPECT_FALSE(modeFrame.isExtended);
    EXPECT_FALSE(modeFrame.isRemoteRequest);
    EXPECT_EQ(modeFrame.dlc, 8u);
    EXPECT_EQ(modeFrame.data[0], kNodeId);
    EXPECT_EQ(modeFrame.data[1], 0x00u);
    EXPECT_EQ(modeFrame.data[2], 0x55u);
    EXPECT_EQ(modeFrame.data[3], 10u);
    EXPECT_EQ(modeFrame.data[4], 0x03u);
    EXPECT_EQ(modeFrame.data[5], 0x00u);
    EXPECT_EQ(modeFrame.data[6], 0x00u);
    EXPECT_EQ(modeFrame.data[7], 0x00u);

    const auto &pMaxFrame = frames[2];
    EXPECT_EQ(pMaxFrame.id, 0x7FFu);
    EXPECT_EQ(pMaxFrame.data[0], kNodeId);
    EXPECT_EQ(pMaxFrame.data[3], 21u);
    EXPECT_EQ(pMaxFrame.data[2], 0x33u);
    EXPECT_EQ(decodeUInt32LE(pMaxFrame, 4), 0u);

    const auto &vMaxFrame = frames[3];
    EXPECT_EQ(vMaxFrame.id, 0x7FFu);
    EXPECT_EQ(vMaxFrame.data[0], kNodeId);
    EXPECT_EQ(vMaxFrame.data[3], 22u);
    EXPECT_EQ(vMaxFrame.data[2], 0x33u);
    EXPECT_EQ(decodeUInt32LE(vMaxFrame, 4), 0u);

    const auto &tMaxFrame = frames[4];
    EXPECT_EQ(tMaxFrame.id, 0x7FFu);
    EXPECT_EQ(tMaxFrame.data[0], kNodeId);
    EXPECT_EQ(tMaxFrame.data[3], 23u);
    EXPECT_EQ(tMaxFrame.data[2], 0x33u);
    EXPECT_EQ(decodeUInt32LE(tMaxFrame, 4), 0u);

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

TEST_F(DamiaoCanTest, UsesReadbackRangesWithoutWritingThem)
{
    auto future = std::async(std::launch::async, [&]() {
        return damiao_.setMode(static_cast<MotorID>(1), CanProtocol::MotorMode::Velocity);
    });
    ackVelocityModeAndFeedbackRanges(1, 6.0f, 20.0f, 5.0f);
    ASSERT_TRUE(future.get());
    transport_->simulateReceive(makeFeedbackFrame(1, 1, 65535, 4095, 4095));
    EXPECT_EQ(damiao_.getPosition(static_cast<MotorID>(1)), 60000);
    EXPECT_EQ(damiao_.getVelocity(static_cast<MotorID>(1)), 200000);
}

TEST_F(DamiaoCanTest, UnconfiguredFeedbackDoesNotClaimModeOrRanges)
{
    damiao_.initializeMotorRefresh({static_cast<MotorID>(1)});
    transport_->simulateReceive(makeFeedbackFrame(1, 0));
    can_driver::SharedDriverState::AxisFeedbackState feedback;
    ASSERT_TRUE(sharedState_->getAxisFeedback(axisKey(static_cast<MotorID>(1)), &feedback));
    EXPECT_FALSE(feedback.modeValid);
    EXPECT_FALSE(feedback.positionValid);
    EXPECT_FALSE(feedback.velocityValid);
    EXPECT_TRUE(feedback.enabledValid);
}

TEST_F(DamiaoCanTest, MissingModeReadbackFailsWithoutRangeWritesOrEnable)
{
    auto future = std::async(std::launch::async, [&]() {
        return damiao_.setMode(static_cast<MotorID>(1), CanProtocol::MotorMode::Velocity);
    });
    ASSERT_TRUE(transport_->waitForSentFrameCount(1, std::chrono::milliseconds(500)));
    transport_->simulateReceive(makeRegisterAck(1, 10, 3));
    EXPECT_FALSE(future.get());
    const auto frames = transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(frames[1].data[2], 0x33);
    EXPECT_EQ(frames[1].data[3], 10);
}

TEST_F(DamiaoCanTest, StopClearsVelocityUsedByKeepalive)
{
    const auto id = static_cast<MotorID>(1);
    configureVelocityMode(id);
    ASSERT_TRUE(damiao_.setVelocity(id, 10000));
    ASSERT_TRUE(damiao_.Stop(id));
    transport_->clearSentFrames();
    ASSERT_TRUE(damiao_.issueRefreshQuery(id, DamiaoCan::RefreshQuery::Keepalive));
    const auto frames = transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_FLOAT_EQ(decodeFloatLE(frames[0]), 0.f);
}

TEST_F(DamiaoCanTest, FailedVelocitySendReturnsFailureAndClearsKeepalive)
{
    const auto id = static_cast<MotorID>(1);
    configureVelocityMode(id);
    transport_->sendResult = CanTransport::SendResult::LinkDown;
    EXPECT_FALSE(damiao_.setVelocity(id, 10000));
    EXPECT_FALSE(damiao_.Stop(id));
    transport_->sendResult = CanTransport::SendResult::Ok;
    transport_->clearSentFrames();
    ASSERT_TRUE(damiao_.issueRefreshQuery(id, DamiaoCan::RefreshQuery::Keepalive));
    ASSERT_EQ(transport_->snapshotSentFrames().size(), 1u);
    EXPECT_FLOAT_EQ(decodeFloatLE(transport_->snapshotSentFrames()[0]), 0.f);
}

TEST_F(DamiaoCanTest, CustomMasterIdRoutesRegisterAndFeedbackReplies)
{
    const auto id = static_cast<MotorID>(7);
    damiao_.setMotorMasterId(id, 8);
    auto future = std::async(std::launch::async, [&]() {
        return damiao_.setMode(id, CanProtocol::MotorMode::Velocity);
    });
    ackVelocityModeAndFeedbackRanges(7, 12.5f, 45.f, 18.f, 8);
    ASSERT_TRUE(future.get());
    auto feedback = makeFeedbackFrame(7, 1);
    transport_->simulateReceive(feedback);
    EXPECT_FALSE(damiao_.isEnabled(id));
    feedback.id = 8;
    transport_->simulateReceive(feedback);
    EXPECT_TRUE(damiao_.isEnabled(id));
}

TEST_F(DamiaoCanTest, InvalidRangeStopsInitializationBeforeEnable)
{
    for (float value : {0.f, -1.f, std::numeric_limits<float>::quiet_NaN(),
                        std::numeric_limits<float>::infinity()}) {
        auto transport = std::make_shared<MockTransport>();
        auto dispatcher = std::make_shared<MockTxDispatcher>(transport);
        DamiaoCan motor(transport, dispatcher);
        auto future = std::async(std::launch::async, [&]() {
            return motor.setMode(static_cast<MotorID>(1), CanProtocol::MotorMode::Velocity);
        });
        for (int i = 0; i < 3; ++i) {
            ASSERT_TRUE(transport->waitForSentFrameCount(i + 1, std::chrono::milliseconds(500)));
            auto ack = makeRegisterAck(1, i < 2 ? 10 : 21,
                                       i < 2 ? 3 : encodeFloatBits(value));
            ack.data[2] = i == 0 ? 0x55 : 0x33;
            transport->simulateReceive(ack);
        }
        EXPECT_FALSE(future.get());
        EXPECT_EQ(transport->snapshotSentFrames().size(), 3u);
    }
}

TEST_F(DamiaoCanTest, DirectCommandExpiryStopsKeepaliveAndRetriesFailedStop)
{
    const auto id = static_cast<MotorID>(1);
    configureVelocityMode(id);
    ASSERT_TRUE(damiao_.setVelocity(id, 10000));
    bool pending = true;
    const auto start = std::chrono::steady_clock::now();
    EXPECT_TRUE(can_driver::directCommandReady(damiao_, id, pending, start, .2,
                                               start + std::chrono::milliseconds(100)));
    transport_->sendResult = CanTransport::SendResult::LinkDown;
    EXPECT_FALSE(can_driver::directCommandReady(damiao_, id, pending, start, .2,
                                                start + std::chrono::milliseconds(300)));
    EXPECT_TRUE(pending);
    transport_->sendResult = CanTransport::SendResult::Ok;
    EXPECT_FALSE(can_driver::directCommandReady(damiao_, id, pending, start, .2,
                                                start + std::chrono::milliseconds(400)));
    EXPECT_FALSE(pending);
    transport_->clearSentFrames();
    ASSERT_TRUE(damiao_.issueRefreshQuery(id, DamiaoCan::RefreshQuery::Keepalive));
    ASSERT_EQ(transport_->snapshotSentFrames().size(), 1u);
    EXPECT_FLOAT_EQ(decodeFloatLE(transport_->snapshotSentFrames()[0]), 0.f);
}

TEST_F(DamiaoCanTest, DisableRequiresNewFeedbackInsteadOfCachedDisabledState)
{
    const auto id = static_cast<MotorID>(1);
    configureVelocityMode(id);
    transport_->simulateReceive(makeFeedbackFrame(1, 0));
    EXPECT_FALSE(damiao_.Disable(id));
}

TEST_F(DamiaoCanTest, MitPositionUsesDocumentedPacketAndNeutralKeepalive)
{
    const auto id = static_cast<MotorID>(7);
    ASSERT_TRUE(damiao_.setMitGains(id, 5, .5));
    auto future = std::async(std::launch::async, [&]() {
        return damiao_.setMode(id, CanProtocol::MotorMode::Position);
    });
    ackVelocityModeAndFeedbackRanges(7, 12.5f, 45.f, 120.f, 0, 1);
    ASSERT_TRUE(future.get());
    transport_->clearSentFrames();
    ASSERT_TRUE(damiao_.setPosition(id, 10000));
    auto frames = transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].id, 7u);
    const std::array<std::uint8_t, 8> target{{0x8a,0x3c,0x7f,0xf0,0x28,0x19,0x97,0xff}};
    EXPECT_EQ(frames[0].data, target);
    transport_->simulateReceive(makeFeedbackFrame(7, 1));
    can_driver::SharedDriverState::AxisFeedbackState feedback;
    ASSERT_TRUE(sharedState_->getAxisFeedback(axisKey(id), &feedback));
    EXPECT_EQ(feedback.mode, CanProtocol::MotorMode::Position);
    EXPECT_TRUE(feedback.modeValid);
    // Switching wire mode while enabled must be rejected without a register write.
    transport_->clearSentFrames();
    EXPECT_FALSE(damiao_.setMode(id, CanProtocol::MotorMode::Velocity));
    EXPECT_TRUE(transport_->snapshotSentFrames().empty());
    ASSERT_TRUE(damiao_.Stop(id));
    transport_->clearSentFrames();
    ASSERT_TRUE(damiao_.issueRefreshQuery(id, DamiaoCan::RefreshQuery::Keepalive));
    frames=transport_->snapshotSentFrames();
    ASSERT_EQ(frames.size(),1u);
    const std::array<std::uint8_t, 8> neutral{{0x7f,0xff,0x7f,0xf0,0,0,7,0xff}};
    EXPECT_EQ(frames[0].id,7u);
    EXPECT_EQ(frames[0].data,neutral);
    EXPECT_FALSE(damiao_.setPosition(id, 130000)); // Outside actual PMAX, no wrap/clamp.
}

TEST_F(DamiaoCanTest, MitRequiresRepresentableNonzeroGains)
{
    const auto id=static_cast<MotorID>(7);
    EXPECT_FALSE(damiao_.setMode(id,CanProtocol::MotorMode::Position));
    EXPECT_FALSE(damiao_.setMitGains(id,5,0));
    EXPECT_FALSE(damiao_.setMitGains(id,5,.0001));
    EXPECT_FALSE(damiao_.setMitGains(id,501,.5));
    EXPECT_TRUE(transport_->snapshotSentFrames().empty());
}
