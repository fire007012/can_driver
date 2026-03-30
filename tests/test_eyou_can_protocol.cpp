#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/EyouCan.h"

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

namespace {

// 轻量 mock：只验证协议层编码/解码，不依赖真实 socketcan。
class MockTransport : public CanTransport {
public:
    void send(const Frame &frame) override
    {
        sentFrames.push_back(frame);
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

class EyouCanTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }

    EyouCanTest()
        : transport(std::make_shared<MockTransport>())
        , eyou(transport)
    {
    }

    std::shared_ptr<MockTransport> transport;
    EyouCan eyou;
};

} // namespace

class EyouCanTestAccessor {
public:
    static void setRefreshState(EyouCan &eyou, const std::vector<uint8_t> &motorIds, bool active)
    {
        std::lock_guard<std::mutex> lock(eyou.refreshMutex);
        eyou.refreshMotorIds = motorIds;
        eyou.managedMotorIds.clear();
        for (const auto motorId : motorIds) {
            eyou.managedMotorIds.insert(motorId);
        }
        eyou.refreshLoopActive.store(active);
        eyou.refreshCycleCount_ = 0;
        std::lock_guard<std::mutex> pendingLock(eyou.pendingReadMutex_);
        eyou.pendingReadRequests_.clear();
    }

    static void refresh(EyouCan &eyou)
    {
        eyou.refreshMotorStates();
    }

    static void ageAllPendingRequests(EyouCan &eyou, std::chrono::milliseconds age)
    {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        for (auto &entry : eyou.pendingReadRequests_) {
            entry.second.inFlight = true;
            entry.second.lastSent = now - age;
        }
    }

    static void expireAllPendingBackoff(EyouCan &eyou)
    {
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        for (auto &entry : eyou.pendingReadRequests_) {
            entry.second.nextEligibleSend = std::chrono::steady_clock::time_point {};
        }
    }

    static std::size_t consecutiveTimeouts(EyouCan &eyou, uint8_t motorId, uint8_t subCommand)
    {
        std::lock_guard<std::mutex> lock(eyou.pendingReadMutex_);
        const auto it = eyou.pendingReadRequests_.find(EyouCan::pendingReadKey(motorId, subCommand));
        return (it == eyou.pendingReadRequests_.end()) ? 0u : it->second.consecutiveTimeouts;
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
    EXPECT_EQ(transport->sentFrames[1].data[0], 0x05);
    EXPECT_EQ(transport->sentFrames[1].data[1], 0x0A);
    EXPECT_EQ(transport->sentFrames[2].data[0], 0x01);
    EXPECT_EQ(transport->sentFrames[2].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[3].data[0], 0x01);
    EXPECT_EQ(transport->sentFrames[3].data[1], 0x0A);
}

TEST_F(EyouCanTest, GetPositionWithoutCacheReturnsZeroWithoutSendingReadRequest)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    const int64_t pos = eyou.getPosition(kMotorId);

    EXPECT_EQ(pos, 0);
    EXPECT_TRUE(transport->sentFrames.empty());
}

TEST_F(EyouCanTest, RefreshDoesNotResendSameReadWhileRequestIsInFlight)
{
    EyouCanTestAccessor::setRefreshState(eyou, {0x05}, true);

    EyouCanTestAccessor::refresh(eyou);
    ASSERT_EQ(transport->sentFrames.size(), 6u);

    EyouCanTestAccessor::refresh(eyou);
    EXPECT_EQ(transport->sentFrames.size(), 6u);
}

TEST_F(EyouCanTest, RefreshBacksOffAfterRepeatedReadTimeouts)
{
    EyouCanTestAccessor::setRefreshState(eyou, {0x05}, true);

    EyouCanTestAccessor::refresh(eyou);
    ASSERT_EQ(transport->sentFrames.size(), 6u);

    EyouCanTestAccessor::ageAllPendingRequests(eyou, std::chrono::milliseconds(25));
    EyouCanTestAccessor::refresh(eyou);
    EXPECT_EQ(transport->sentFrames.size(), 6u);
    EXPECT_EQ(EyouCanTestAccessor::consecutiveTimeouts(eyou, 0x05, 0x07), 1u);

    EyouCanTestAccessor::expireAllPendingBackoff(eyou);
    EyouCanTestAccessor::refresh(eyou);
    EXPECT_EQ(transport->sentFrames.size(), 8u);
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

    // 速度 -40000 -> 低于 int16 下限，应钳制为 -32768。
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
    EXPECT_EQ(eyou.getVelocity(static_cast<MotorID>(0x05)), -32768);
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

TEST_F(EyouCanTest, DISABLED_TODO_HandleWriteAckModeAndEnable)
{
    GTEST_SKIP() << "TODO: cover write ack (0x02) branches for mode/enable.";
}

TEST_F(EyouCanTest, DISABLED_TODO_HandleErrorCodeReadResponse)
{
    GTEST_SKIP() << "TODO: cover read response (0x15) error-report branch.";
}
