#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/EyouCan.h"

#include <cstdint>
#include <memory>
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

TEST_F(EyouCanTest, SetPositionEncodesExpectedWriteFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);
    constexpr int32_t kPosition = 0x01020304;

    ASSERT_TRUE(eyou.setPosition(kMotorId, kPosition));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    // Eyou/PP 写命令：0x01 + 子命令 + 大端负载。
    EXPECT_EQ(frame.id, 0x0005u);
    EXPECT_FALSE(frame.isExtended);
    EXPECT_FALSE(frame.isRemoteRequest);
    // 新协议实现统一输出 8 字节帧，尾部用 0 填充。
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0x01); // write
    EXPECT_EQ(frame.data[1], 0x0A); // position sub-command
    EXPECT_EQ(frame.data[2], 0x01);
    EXPECT_EQ(frame.data[3], 0x02);
    EXPECT_EQ(frame.data[4], 0x03);
    EXPECT_EQ(frame.data[5], 0x04);
    EXPECT_EQ(frame.data[6], 0x00);
    EXPECT_EQ(frame.data[7], 0x00);
}

TEST_F(EyouCanTest, GetPositionWithoutCacheSendsReadRequest)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    const int64_t pos = eyou.getPosition(kMotorId);

    EXPECT_EQ(pos, 0);
    ASSERT_EQ(transport->sentFrames.size(), 1u);
    const auto &frame = transport->sentFrames[0];
    // 首次读取位置应主动发起寄存器读取（0x03/0x07）。
    EXPECT_EQ(frame.id, 0x0005u);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0x03); // read
    EXPECT_EQ(frame.data[1], 0x07); // position register
    EXPECT_EQ(frame.data[2], 0x00);
    EXPECT_EQ(frame.data[7], 0x00);
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
