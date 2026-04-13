#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/EyouCan.h"
#include "can_driver/SharedDriverState.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace {

// 轻量 mock：只验证协议层编码/解码，不依赖真实 socketcan。
class MockTransport : public CanTransport {
public:
    SendResult send(const Frame &frame) override
    {
        sentFrames.push_back(frame);
        return SendResult::Ok;
    }

    std::size_t addReceiveHandler(ReceiveHandler handler) override
    {
        receiveHandler = std::move(handler);
        return 1;
    }

    void removeReceiveHandler(std::size_t) override
    {
        receiveHandler = nullptr;
    }

    void simulateReceive(const Frame &frame) const
    {
        if (receiveHandler) {
            receiveHandler(frame);
        }
    }

    void clearSent()
    {
        sentFrames.clear();
    }

    std::vector<Frame> sentFrames;
    ReceiveHandler receiveHandler;
};

class MockTxDispatcher : public CanTxDispatcher {
public:
    explicit MockTxDispatcher(std::shared_ptr<MockTransport> transport)
        : transport(std::move(transport))
    {
    }

    void submit(const Request &request) override
    {
        requests.push_back(request);
        if (autoSend && transport) {
            const auto eventTime = std::chrono::steady_clock::now();
            const auto result = transport->send(request.frame);
            if (request.completion) {
                request.completion(true, result, eventTime);
            }
        }
    }

    std::shared_ptr<MockTransport> transport;
    std::vector<Request> requests;
    bool autoSend{true};
};

class EyouCanTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }

    EyouCanTest()
        : transport(std::make_shared<MockTransport>())
        , txDispatcher(std::make_shared<MockTxDispatcher>(transport))
        , sharedState(std::make_shared<can_driver::SharedDriverState>())
        , eyou(transport, txDispatcher, sharedState, "can0")
    {
    }

    std::shared_ptr<MockTransport> transport;
    std::shared_ptr<MockTxDispatcher> txDispatcher;
    std::shared_ptr<can_driver::SharedDriverState> sharedState;
    EyouCan eyou;
};

} // namespace

class EyouCanTestAccessor {
public:
    static bool isQueued(EyouCan &eyou, uint8_t motorId, uint8_t subCommand)
    {
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        const auto it = eyou.pendingReadRequests_.find(EyouCan::pendingReadKey(motorId, subCommand));
        return (it == eyou.pendingReadRequests_.end()) ? false : it->second.queued;
    }

    static bool isInFlight(EyouCan &eyou, uint8_t motorId, uint8_t subCommand)
    {
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        const auto it = eyou.pendingReadRequests_.find(EyouCan::pendingReadKey(motorId, subCommand));
        return (it == eyou.pendingReadRequests_.end()) ? false : it->second.inFlight;
    }

    static void setLastResponseAge(EyouCan &eyou,
                                   uint8_t motorId,
                                   uint8_t subCommand,
                                   std::chrono::milliseconds age)
    {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        auto &request = eyou.pendingReadRequests_[EyouCan::pendingReadKey(motorId, subCommand)];
        request.lastResponse = now - age;
    }

    static void setLastSentAge(EyouCan &eyou,
                               uint8_t motorId,
                               uint8_t subCommand,
                               std::chrono::milliseconds age)
    {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        auto &request = eyou.pendingReadRequests_[EyouCan::pendingReadKey(motorId, subCommand)];
        request.lastSent = now - age;
        request.inFlight = true;
    }

    static void setMissedRefreshWindows(EyouCan &eyou,
                                        uint8_t motorId,
                                        uint8_t subCommand,
                                        std::size_t windows)
    {
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        auto &request = eyou.pendingReadRequests_[EyouCan::pendingReadKey(motorId, subCommand)];
        request.missedRefreshWindows = windows;
    }

    static std::size_t missedRefreshWindows(EyouCan &eyou, uint8_t motorId, uint8_t subCommand)
    {
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        const auto it = eyou.pendingReadRequests_.find(EyouCan::pendingReadKey(motorId, subCommand));
        return (it == eyou.pendingReadRequests_.end()) ? 0u : it->second.missedRefreshWindows;
    }

    static void setWarnedStaleBuckets(EyouCan &eyou,
                                      uint8_t motorId,
                                      uint8_t subCommand,
                                      std::size_t buckets)
    {
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        auto &request = eyou.pendingReadRequests_[EyouCan::pendingReadKey(motorId, subCommand)];
        request.warnedStaleBuckets = buckets;
    }

    static std::size_t warnedStaleBuckets(EyouCan &eyou,
                                          uint8_t motorId,
                                          uint8_t subCommand)
    {
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        const auto it = eyou.pendingReadRequests_.find(EyouCan::pendingReadKey(motorId, subCommand));
        return (it == eyou.pendingReadRequests_.end()) ? 0u : it->second.warnedStaleBuckets;
    }

    static std::size_t staleWarnWindowThreshold(uint8_t subCommand)
    {
        return EyouCan::feedbackStaleWarnWindowThreshold(subCommand);
    }

    static std::chrono::milliseconds staleWarnThreshold(uint8_t subCommand)
    {
        return EyouCan::feedbackStaleWarnThreshold(subCommand);
    }
};

TEST_F(EyouCanTest, SetPositionEncodesExpectedWriteFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);
    constexpr int32_t kPosition = 0x01020304;

    ASSERT_TRUE(eyou.setPosition(kMotorId, kPosition));
    ASSERT_EQ(transport->sentFrames.size(), 2u);

    const auto &velocityFrame = transport->sentFrames[0];
    const auto &positionFrame = transport->sentFrames[1];

    // Eyou/PP 位置命令会先补一帧目标速度，再发目标位置。
    EXPECT_EQ(velocityFrame.id, 0x0005u);
    EXPECT_FALSE(velocityFrame.isExtended);
    EXPECT_FALSE(velocityFrame.isRemoteRequest);
    EXPECT_EQ(velocityFrame.dlc, 8u);
    EXPECT_EQ(velocityFrame.data[0], 0x01);
    EXPECT_EQ(velocityFrame.data[1], 0x09);
    EXPECT_EQ(velocityFrame.data[2], 0x00);
    EXPECT_EQ(velocityFrame.data[3], 0x00);
    EXPECT_EQ(velocityFrame.data[4], 0x2A);
    EXPECT_EQ(velocityFrame.data[5], 0xAA);

    EXPECT_EQ(positionFrame.id, 0x0005u);
    EXPECT_FALSE(positionFrame.isExtended);
    EXPECT_FALSE(positionFrame.isRemoteRequest);
    EXPECT_EQ(positionFrame.dlc, 8u);
    EXPECT_EQ(positionFrame.data[0], 0x01); // write
    EXPECT_EQ(positionFrame.data[1], 0x0A); // position sub-command
    EXPECT_EQ(positionFrame.data[2], 0x01);
    EXPECT_EQ(positionFrame.data[3], 0x02);
    EXPECT_EQ(positionFrame.data[4], 0x03);
    EXPECT_EQ(positionFrame.data[5], 0x04);
    EXPECT_EQ(positionFrame.data[6], 0x00);
    EXPECT_EQ(positionFrame.data[7], 0x00);
}

TEST_F(EyouCanTest, FastWriteModeUsesCmd05ForVelocityAndPosition)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    eyou.setFastWriteEnabled(true);

    ASSERT_TRUE(eyou.setVelocity(kMotorId, 123));
    ASSERT_EQ(transport->sentFrames.size(), 2u);
    EXPECT_EQ(transport->sentFrames[0].data[0], 0x05);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[1].data[0], 0x01);
    EXPECT_EQ(transport->sentFrames[1].data[1], 0x09);

    transport->clearSent();
    ASSERT_TRUE(eyou.setPosition(kMotorId, 456));
    ASSERT_EQ(transport->sentFrames.size(), 4u);
    EXPECT_EQ(transport->sentFrames[0].data[0], 0x05);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[1].data[0], 0x01);
    EXPECT_EQ(transport->sentFrames[1].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[2].data[0], 0x05);
    EXPECT_EQ(transport->sentFrames[2].data[1], 0x0A);
    EXPECT_EQ(transport->sentFrames[3].data[0], 0x01);
    EXPECT_EQ(transport->sentFrames[3].data[1], 0x0A);
}

TEST_F(EyouCanTest, PositionVelocityDefaultCanBeOverridden)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    eyou.setDefaultPositionVelocityRaw(0x00001234);

    ASSERT_TRUE(eyou.setPosition(kMotorId, 456));
    ASSERT_EQ(transport->sentFrames.size(), 2u);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[0].data[2], 0x00);
    EXPECT_EQ(transport->sentFrames[0].data[3], 0x00);
    EXPECT_EQ(transport->sentFrames[0].data[4], 0x12);
    EXPECT_EQ(transport->sentFrames[0].data[5], 0x34);
    EXPECT_EQ(transport->sentFrames[1].data[1], 0x0A);
}

TEST_F(EyouCanTest, PositionAndCspVelocityDefaultsCanDiffer)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    eyou.setDefaultPositionVelocityRaw(0x00001234);
    eyou.setDefaultCspVelocityRaw(0x00005678);

    ASSERT_TRUE(eyou.setPosition(kMotorId, 456));
    ASSERT_EQ(transport->sentFrames.size(), 2u);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[0].data[4], 0x12);
    EXPECT_EQ(transport->sentFrames[0].data[5], 0x34);

    transport->clearSent();
    txDispatcher->requests.clear();

    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 789));
    ASSERT_EQ(transport->sentFrames.size(), 2u);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[0].data[4], 0x56);
    EXPECT_EQ(transport->sentFrames[0].data[5], 0x78);
    EXPECT_EQ(transport->sentFrames[1].data[1], 0x0A);
}

TEST_F(EyouCanTest, PerMotorPositionVelocityDefaultOverridesGlobalDefault)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    eyou.setDefaultPositionVelocityRaw(0x00001234);
    eyou.setMotorDefaultPositionVelocityRaw(kMotorId, 0x00004321);

    ASSERT_TRUE(eyou.setPosition(kMotorId, 456));
    ASSERT_EQ(transport->sentFrames.size(), 2u);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[0].data[4], 0x43);
    EXPECT_EQ(transport->sentFrames[0].data[5], 0x21);
}

TEST_F(EyouCanTest, WritesRouteThroughUnifiedTxDispatcher)
{
    ASSERT_TRUE(eyou.setPosition(static_cast<MotorID>(0x05), 123));
    ASSERT_EQ(txDispatcher->requests.size(), 2u);
    EXPECT_EQ(txDispatcher->requests[0].category, CanTxDispatcher::Category::Control);
    EXPECT_STREQ(txDispatcher->requests[0].source, "EyouCan::sendWriteCommand");
    EXPECT_EQ(txDispatcher->requests[1].category, CanTxDispatcher::Category::Control);
}

TEST_F(EyouCanTest, CommandsPopulateSharedStateIntentAndTargets)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);
    const auto axisKey = can_driver::MakeAxisKey("can0", CanType::PP, kMotorId);

    ASSERT_TRUE(eyou.setPosition(kMotorId, 1234));
    ASSERT_TRUE(eyou.Enable(kMotorId));

    can_driver::SharedDriverState::AxisCommandState command;
    ASSERT_TRUE(sharedState->getAxisCommand(axisKey, &command));
    EXPECT_EQ(command.targetPosition, 1234);
    EXPECT_EQ(command.targetVelocity, 0);
    EXPECT_EQ(command.desiredMode, CanProtocol::MotorMode::Position);
    EXPECT_TRUE(command.desiredModeValid);
    EXPECT_TRUE(command.valid);
    EXPECT_EQ(sharedState->getAxisIntent(axisKey), can_driver::AxisIntent::Enable);
}

TEST_F(EyouCanTest, SetModeUpdatesDesiredModeWithoutPretendingMotionCommandExists)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);
    const auto axisKey = can_driver::MakeAxisKey("can0", CanType::PP, kMotorId);

    ASSERT_TRUE(eyou.setPosition(kMotorId, 1234));
    ASSERT_TRUE(eyou.setMode(kMotorId, CanProtocol::MotorMode::CSP));

    can_driver::SharedDriverState::AxisCommandState command;
    ASSERT_TRUE(sharedState->getAxisCommand(axisKey, &command));
    EXPECT_EQ(command.desiredMode, CanProtocol::MotorMode::CSP);
    EXPECT_TRUE(command.desiredModeValid);
    EXPECT_FALSE(command.valid);
    EXPECT_EQ(command.targetPosition, 0);
    EXPECT_EQ(command.targetVelocity, 0);
    EXPECT_EQ(command.lastCommandSteadyNs, 0);
}

TEST_F(EyouCanTest, IssueRefreshQueryMapsEnumsToExpectedSubcommands)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    eyou.issueRefreshQuery(kMotorId, EyouCan::RefreshQuery::Position);
    eyou.issueRefreshQuery(kMotorId, EyouCan::RefreshQuery::Velocity);
    eyou.issueRefreshQuery(kMotorId, EyouCan::RefreshQuery::Mode);
    eyou.issueRefreshQuery(kMotorId, EyouCan::RefreshQuery::Enable);
    eyou.issueRefreshQuery(kMotorId, EyouCan::RefreshQuery::Fault);
    eyou.issueRefreshQuery(kMotorId, EyouCan::RefreshQuery::Current);

    ASSERT_EQ(txDispatcher->requests.size(), 6u);
    ASSERT_EQ(transport->sentFrames.size(), 6u);
    for (const auto &request : txDispatcher->requests) {
        EXPECT_EQ(request.category, CanTxDispatcher::Category::Query);
        EXPECT_STREQ(request.source, "EyouCan::sendReadCommand");
    }
    for (const auto &frame : transport->sentFrames) {
        EXPECT_EQ(frame.dlc, 2u);
    }
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x07u);
    EXPECT_EQ(transport->sentFrames[1].data[1], 0x06u);
    EXPECT_EQ(transport->sentFrames[2].data[1], 0x0Fu);
    EXPECT_EQ(transport->sentFrames[3].data[1], 0x10u);
    EXPECT_EQ(transport->sentFrames[4].data[1], 0x15u);
    EXPECT_EQ(transport->sentFrames[5].data[1], 0x05u);
}

TEST_F(EyouCanTest, GetPositionWithoutCacheReturnsZeroWithoutSendingReadRequest)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    const int64_t pos = eyou.getPosition(kMotorId);

    EXPECT_EQ(pos, 0);
    EXPECT_TRUE(transport->sentFrames.empty());
}

TEST_F(EyouCanTest, IssueRefreshQueryKeepsSendingWhenPreviousReadStillHasNoResponse)
{
    eyou.issueRefreshQuery(static_cast<MotorID>(0x05), EyouCan::RefreshQuery::Position);
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    eyou.issueRefreshQuery(static_cast<MotorID>(0x05), EyouCan::RefreshQuery::Position);
    EXPECT_EQ(transport->sentFrames.size(), 2u);
}

TEST_F(EyouCanTest, QueuedReadRequestDoesNotDuplicateSubmitBeforeDispatcherCompletion)
{
    txDispatcher->autoSend = false;

    eyou.issueRefreshQuery(static_cast<MotorID>(0x05), EyouCan::RefreshQuery::Position);
    ASSERT_EQ(txDispatcher->requests.size(), 1u);
    EXPECT_TRUE(EyouCanTestAccessor::isQueued(eyou, 0x05, 0x07));
    EXPECT_FALSE(EyouCanTestAccessor::isInFlight(eyou, 0x05, 0x07));

    eyou.issueRefreshQuery(static_cast<MotorID>(0x05), EyouCan::RefreshQuery::Position);

    EXPECT_EQ(txDispatcher->requests.size(), 1u);
    EXPECT_TRUE(EyouCanTestAccessor::isQueued(eyou, 0x05, 0x07));
}

TEST_F(EyouCanTest, RepeatedRefreshQueryIncrementsMissedWindowsWithoutBackoff)
{
    eyou.issueRefreshQuery(static_cast<MotorID>(0x05), EyouCan::RefreshQuery::Position);
    ASSERT_EQ(transport->sentFrames.size(), 1u);
    EXPECT_EQ(EyouCanTestAccessor::missedRefreshWindows(eyou, 0x05, 0x07), 1u);

    eyou.issueRefreshQuery(static_cast<MotorID>(0x05), EyouCan::RefreshQuery::Position);
    EXPECT_EQ(transport->sentFrames.size(), 2u);
    EXPECT_EQ(EyouCanTestAccessor::missedRefreshWindows(eyou, 0x05, 0x07), 2u);
}

TEST_F(EyouCanTest, SlowRefreshRateDoesNotSuppressRepeatedRefreshQueries)
{
    eyou.setRefreshRateHz(5.0);
    eyou.initializeMotorRefresh({static_cast<MotorID>(0x05), static_cast<MotorID>(0x06), static_cast<MotorID>(0x07)});

    eyou.issueRefreshQuery(static_cast<MotorID>(0x05), EyouCan::RefreshQuery::Position);
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    eyou.issueRefreshQuery(static_cast<MotorID>(0x05), EyouCan::RefreshQuery::Position);
    EXPECT_EQ(transport->sentFrames.size(), 2u);
}

TEST_F(EyouCanTest, StaleWarningUsesFeedbackAgeEvenWithoutPriorMissedWindowCounter)
{
    constexpr uint8_t kMotorId = 0x05;
    constexpr uint8_t kSubCommand = 0x07;

    EyouCanTestAccessor::setLastResponseAge(
        eyou,
        kMotorId,
        kSubCommand,
        EyouCanTestAccessor::staleWarnThreshold(kSubCommand) + std::chrono::milliseconds(50));
    EyouCanTestAccessor::setMissedRefreshWindows(eyou, kMotorId, kSubCommand, 0u);

    ASSERT_TRUE(eyou.issueRefreshQuery(static_cast<MotorID>(kMotorId), EyouCan::RefreshQuery::Position));
    EXPECT_EQ(EyouCanTestAccessor::warnedStaleBuckets(eyou, kMotorId, kSubCommand), 1u);
    EXPECT_EQ(EyouCanTestAccessor::missedRefreshWindows(eyou, kMotorId, kSubCommand), 1u);
}

TEST_F(EyouCanTest, StaleWarningStaysQuietBeforeFeedbackAgeThreshold)
{
    constexpr uint8_t kMotorId = 0x05;
    constexpr uint8_t kSubCommand = 0x07;
    const auto threshold = EyouCanTestAccessor::staleWarnThreshold(kSubCommand);
    ASSERT_GT(threshold.count(), 0);

    EyouCanTestAccessor::setLastResponseAge(eyou, kMotorId, kSubCommand, threshold - std::chrono::milliseconds(1));
    EyouCanTestAccessor::setMissedRefreshWindows(eyou, kMotorId, kSubCommand, 0u);

    ASSERT_TRUE(eyou.issueRefreshQuery(static_cast<MotorID>(kMotorId), EyouCan::RefreshQuery::Position));
    EXPECT_EQ(EyouCanTestAccessor::warnedStaleBuckets(eyou, kMotorId, kSubCommand), 0u);
    EXPECT_EQ(EyouCanTestAccessor::missedRefreshWindows(eyou, kMotorId, kSubCommand), 1u);
}

TEST_F(EyouCanTest, StaleWarningUsesFirstSendAgeWhenNoResponseHasEverArrived)
{
    constexpr uint8_t kMotorId = 0x05;
    constexpr uint8_t kSubCommand = 0x07;
    const auto threshold = EyouCanTestAccessor::staleWarnThreshold(kSubCommand);

    EyouCanTestAccessor::setLastSentAge(
        eyou, kMotorId, kSubCommand, threshold + std::chrono::milliseconds(25));

    ASSERT_TRUE(eyou.issueRefreshQuery(static_cast<MotorID>(kMotorId), EyouCan::RefreshQuery::Position));
    EXPECT_EQ(EyouCanTestAccessor::warnedStaleBuckets(eyou, kMotorId, kSubCommand), 1u);
}

TEST_F(EyouCanTest, ReadResponseResetsMissedRefreshWindowTracking)
{
    constexpr uint8_t kMotorId = 0x05;
    constexpr uint8_t kSubCommand = 0x07;
    const auto threshold = EyouCanTestAccessor::staleWarnThreshold(kSubCommand);
    ASSERT_GT(threshold.count(), 0);

    EyouCanTestAccessor::setLastResponseAge(eyou, kMotorId, kSubCommand, threshold + std::chrono::milliseconds(50));
    EyouCanTestAccessor::setMissedRefreshWindows(
        eyou, kMotorId, kSubCommand, EyouCanTestAccessor::staleWarnWindowThreshold(kSubCommand));
    EyouCanTestAccessor::setWarnedStaleBuckets(eyou, kMotorId, kSubCommand, 1u);

    CanTransport::Frame frame {};
    frame.id = kMotorId;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 6;
    frame.data[0] = 0x04;
    frame.data[1] = kSubCommand;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    frame.data[5] = 0xE8;

    transport->simulateReceive(frame);

    EXPECT_EQ(EyouCanTestAccessor::missedRefreshWindows(eyou, kMotorId, kSubCommand), 0u);
    EXPECT_EQ(EyouCanTestAccessor::warnedStaleBuckets(eyou, kMotorId, kSubCommand), 0u);
}

TEST_F(EyouCanTest, HandleReadResponseUpdatesPositionCache)
{
    // 读取返回帧：0x04 + 0x07 + 4 字节大端值。
    CanTransport::Frame frame {};
    frame.id = 0x0005;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 6;
    frame.data[0] = 0x04; // read response
    frame.data[1] = 0x07; // position register
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    frame.data[5] = 0xE8; // 1000 (big endian)

    transport->simulateReceive(frame);

    EXPECT_EQ(eyou.getPosition(static_cast<MotorID>(0x05)), 1000);
}

TEST_F(EyouCanTest, ReadResponsesUpdateSharedFeedbackFreshness)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);
    const auto axisKey = can_driver::MakeAxisKey("can0", CanType::PP, kMotorId);

    CanTransport::Frame frame {};
    frame.id = 0x0005;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 6;
    frame.data[0] = 0x04;
    frame.data[1] = 0x07;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    frame.data[5] = 0xE8;

    transport->simulateReceive(frame);

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    ASSERT_TRUE(sharedState->getAxisFeedback(axisKey, &feedback));
    EXPECT_TRUE(feedback.feedbackSeen);
    EXPECT_TRUE(feedback.positionValid);
    EXPECT_EQ(feedback.position, 1000);
    EXPECT_EQ(feedback.consecutiveTimeoutCount, 0u);
    EXPECT_GT(feedback.lastRxSteadyNs, 0);
}

TEST_F(EyouCanTest, ModeReadResponseDecodesCspAndUpdatesSharedFeedback)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);
    const auto axisKey = can_driver::MakeAxisKey("can0", CanType::PP, kMotorId);

    CanTransport::Frame frame {};
    frame.id = 0x0005;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 6;
    frame.data[0] = 0x04;
    frame.data[1] = 0x0F;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x05;

    transport->simulateReceive(frame);

    can_driver::SharedDriverState::AxisFeedbackState feedback;
    ASSERT_TRUE(sharedState->getAxisFeedback(axisKey, &feedback));
    EXPECT_TRUE(feedback.feedbackSeen);
    EXPECT_EQ(feedback.mode, CanProtocol::MotorMode::CSP);
}

TEST_F(EyouCanTest, HandleResponseIgnoresExtendedFrame)
{
    // 协议仅支持标准帧，扩展帧必须被忽略。
    CanTransport::Frame frame {};
    frame.id = 0x0005;
    frame.isExtended = true;
    frame.isRemoteRequest = false;
    frame.dlc = 6;
    frame.data[0] = 0x04;
    frame.data[1] = 0x07;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    frame.data[5] = 0xE8;

    transport->simulateReceive(frame);

    EXPECT_EQ(eyou.getPosition(static_cast<MotorID>(0x05)), 0);
}

TEST_F(EyouCanTest, HandleResponseIgnoresNonEyouCanIdRange)
{
    // 0x241 属于其他协议响应 ID，Eyou 应静默忽略。
    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 6;
    frame.data[0] = 0x04;
    frame.data[1] = 0x07;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    frame.data[5] = 0xE8;

    transport->simulateReceive(frame);

    EXPECT_EQ(eyou.getPosition(static_cast<MotorID>(0x41)), 0);
}

TEST_F(EyouCanTest, HandleResponseIgnoresUnmanagedMotorId)
{
    // 先注册受管电机 0x05，使过滤逻辑进入白名单模式。
    ASSERT_TRUE(eyou.setPosition(static_cast<MotorID>(0x05), 123));
    transport->clearSent();

    CanTransport::Frame frame {};
    frame.id = 0x0006;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.dlc = 6;
    frame.data[0] = 0x04;
    frame.data[1] = 0x07;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x03;
    frame.data[5] = 0xE8;

    transport->simulateReceive(frame);

    EXPECT_EQ(eyou.getPosition(static_cast<MotorID>(0x06)), 0);
}

TEST_F(EyouCanTest, GetCurrentAndVelocityClampInt16Range)
{
    // 电流 40000 -> 超过 int16 上限，应钳制为 32767。
    CanTransport::Frame currentFrame {};
    currentFrame.id = 0x0005;
    currentFrame.isExtended = false;
    currentFrame.isRemoteRequest = false;
    currentFrame.dlc = 6;
    currentFrame.data[0] = 0x04;
    currentFrame.data[1] = 0x05;
    currentFrame.data[2] = 0x00;
    currentFrame.data[3] = 0x00;
    currentFrame.data[4] = 0x9C;
    currentFrame.data[5] = 0x40;
    transport->simulateReceive(currentFrame);

    // 速度字段现已发布为 int32，应保留完整反馈值。
    CanTransport::Frame velocityFrame {};
    velocityFrame.id = 0x0005;
    velocityFrame.isExtended = false;
    velocityFrame.isRemoteRequest = false;
    velocityFrame.dlc = 6;
    velocityFrame.data[0] = 0x04;
    velocityFrame.data[1] = 0x06;
    velocityFrame.data[2] = 0xFF;
    velocityFrame.data[3] = 0xFF;
    velocityFrame.data[4] = 0x63;
    velocityFrame.data[5] = 0xC0;
    transport->simulateReceive(velocityFrame);

    EXPECT_EQ(eyou.getCurrent(static_cast<MotorID>(0x05)), 32767);
    EXPECT_EQ(eyou.getVelocity(static_cast<MotorID>(0x05)), -40000);
}

TEST_F(EyouCanTest, ReadResponsesUpdateEnabledAndFaultState)
{
    // enabled = 1
    CanTransport::Frame enableFrame {};
    enableFrame.id = 0x0005;
    enableFrame.isExtended = false;
    enableFrame.isRemoteRequest = false;
    enableFrame.dlc = 6;
    enableFrame.data[0] = 0x04;
    enableFrame.data[1] = 0x10;
    enableFrame.data[2] = 0x00;
    enableFrame.data[3] = 0x00;
    enableFrame.data[4] = 0x00;
    enableFrame.data[5] = 0x01;
    transport->simulateReceive(enableFrame);

    CanTransport::Frame faultFrame {};
    faultFrame.id = 0x0005;
    faultFrame.isExtended = false;
    faultFrame.isRemoteRequest = false;
    faultFrame.dlc = 6;
    faultFrame.data[0] = 0x04;
    faultFrame.data[1] = 0x15;
    faultFrame.data[2] = 0x00;
    faultFrame.data[3] = 0x00;
    faultFrame.data[4] = 0x00;
    faultFrame.data[5] = 0x01;
    transport->simulateReceive(faultFrame);

    EXPECT_TRUE(eyou.isEnabled(static_cast<MotorID>(0x05)));
    EXPECT_TRUE(eyou.hasFault(static_cast<MotorID>(0x05)));
}

TEST_F(EyouCanTest, ReadSerialNumberRequestsSubCommand02AndCachesResponse)
{
    uint32_t serialNumber = 0;

    std::thread responder([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CanTransport::Frame frame {};
        frame.id = 0x0005;
        frame.isExtended = false;
        frame.isRemoteRequest = false;
        frame.dlc = 6;
        frame.data[0] = 0x04;
        frame.data[1] = 0x02;
        frame.data[2] = 0x12;
        frame.data[3] = 0x34;
        frame.data[4] = 0x56;
        frame.data[5] = 0x78;
        transport->simulateReceive(frame);
    });

    ASSERT_TRUE(eyou.readSerialNumber(static_cast<MotorID>(0x05), &serialNumber));
    EXPECT_EQ(serialNumber, 0x12345678u);
    ASSERT_FALSE(transport->sentFrames.empty());
    EXPECT_EQ(transport->sentFrames.back().data[0], 0x03);
    EXPECT_EQ(transport->sentFrames.back().data[1], 0x02);

    responder.join();
}

TEST_F(EyouCanTest, PersistParametersUsesSerialNumberLow24BitsWithSaveFlag)
{
    std::thread responder([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CanTransport::Frame frame {};
        frame.id = 0x0005;
        frame.isExtended = false;
        frame.isRemoteRequest = false;
        frame.dlc = 6;
        frame.data[0] = 0x04;
        frame.data[1] = 0x02;
        frame.data[2] = 0x12;
        frame.data[3] = 0x34;
        frame.data[4] = 0x56;
        frame.data[5] = 0x78;
        transport->simulateReceive(frame);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CanTransport::Frame ack {};
        ack.id = 0x0005;
        ack.isExtended = false;
        ack.isRemoteRequest = false;
        ack.dlc = 3;
        ack.data[0] = 0x02;
        ack.data[1] = 0x4D;
        ack.data[2] = 0x01;
        transport->simulateReceive(ack);
    });

    ASSERT_TRUE(eyou.persistParameters(static_cast<MotorID>(0x05)));
    ASSERT_GE(transport->sentFrames.size(), 2u);

    const auto &readFrame = transport->sentFrames[0];
    EXPECT_EQ(readFrame.data[0], 0x03);
    EXPECT_EQ(readFrame.data[1], 0x02);

    const auto &writeFrame = transport->sentFrames.back();
    EXPECT_EQ(writeFrame.data[0], 0x01);
    EXPECT_EQ(writeFrame.data[1], 0x4D);
    EXPECT_EQ(writeFrame.data[2], 0x34);
    EXPECT_EQ(writeFrame.data[3], 0x56);
    EXPECT_EQ(writeFrame.data[4], 0x78);
    EXPECT_EQ(writeFrame.data[5], 0x01);

    responder.join();
}

TEST_F(EyouCanTest, PersistParametersFailsWhenWriteAckReportsFailure)
{
    std::thread responder([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CanTransport::Frame frame {};
        frame.id = 0x0005;
        frame.isExtended = false;
        frame.isRemoteRequest = false;
        frame.dlc = 6;
        frame.data[0] = 0x04;
        frame.data[1] = 0x02;
        frame.data[2] = 0x12;
        frame.data[3] = 0x34;
        frame.data[4] = 0x56;
        frame.data[5] = 0x78;
        transport->simulateReceive(frame);

        for (int i = 0; i < 3; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10 + 50 * i));
            CanTransport::Frame ack {};
            ack.id = 0x0005;
            ack.isExtended = false;
            ack.isRemoteRequest = false;
            ack.dlc = 3;
            ack.data[0] = 0x02;
            ack.data[1] = 0x4D;
            ack.data[2] = 0x00;
            transport->simulateReceive(ack);
        }
    });

    EXPECT_FALSE(eyou.persistParameters(static_cast<MotorID>(0x05)));

    responder.join();
}

TEST_F(EyouCanTest, PersistParametersRetriesAfterRejectedAck)
{
    std::thread responder([this]() {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CanTransport::Frame frame {};
        frame.id = 0x0005;
        frame.isExtended = false;
        frame.isRemoteRequest = false;
        frame.dlc = 6;
        frame.data[0] = 0x04;
        frame.data[1] = 0x02;
        frame.data[2] = 0x12;
        frame.data[3] = 0x34;
        frame.data[4] = 0x56;
        frame.data[5] = 0x78;
        transport->simulateReceive(frame);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        CanTransport::Frame ack0 {};
        ack0.id = 0x0005;
        ack0.isExtended = false;
        ack0.isRemoteRequest = false;
        ack0.dlc = 3;
        ack0.data[0] = 0x02;
        ack0.data[1] = 0x4D;
        ack0.data[2] = 0x00;
        transport->simulateReceive(ack0);

        std::this_thread::sleep_for(std::chrono::milliseconds(60));
        CanTransport::Frame ack1 {};
        ack1.id = 0x0005;
        ack1.isExtended = false;
        ack1.isRemoteRequest = false;
        ack1.dlc = 3;
        ack1.data[0] = 0x02;
        ack1.data[1] = 0x4D;
        ack1.data[2] = 0x01;
        transport->simulateReceive(ack1);
    });

    EXPECT_TRUE(eyou.persistParameters(static_cast<MotorID>(0x05)));

    responder.join();
}

TEST_F(EyouCanTest, DISABLED_TODO_HandleWriteAckModeAndEnable)
{
    GTEST_SKIP() << "TODO: cover write ack (0x02) branches for mode/enable.";
}

TEST_F(EyouCanTest, DISABLED_TODO_HandleErrorCodeReadResponse)
{
    GTEST_SKIP() << "TODO: cover read response (0x15) error-report branch.";
}
