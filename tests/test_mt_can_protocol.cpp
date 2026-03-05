#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/MtCan.h"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace {

// Mock 传输层，隔离 MtCan 协议编解码逻辑。
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

class MtCanTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }

    MtCanTest()
        : transport(std::make_shared<MockTransport>())
        , mt(transport)
    {
    }

    std::shared_ptr<MockTransport> transport;
    MtCan mt;
};

} // namespace

TEST_F(MtCanTest, SetVelocityEncodesExpectedFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    constexpr int32_t kVelocity = -123456;

    ASSERT_TRUE(mt.setVelocity(kMotorId, kVelocity));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    // MT 速度命令使用 0xA2，速度值小端写入 data[4..7]。
    EXPECT_EQ(frame.id, 0x141u);
    EXPECT_FALSE(frame.isExtended);
    EXPECT_FALSE(frame.isRemoteRequest);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0xA2);
    EXPECT_EQ(frame.data[4], static_cast<uint8_t>(kVelocity & 0xFF));
    EXPECT_EQ(frame.data[5], static_cast<uint8_t>((kVelocity >> 8) & 0xFF));
    EXPECT_EQ(frame.data[6], static_cast<uint8_t>((kVelocity >> 16) & 0xFF));
    EXPECT_EQ(frame.data[7], static_cast<uint8_t>((kVelocity >> 24) & 0xFF));
}

TEST_F(MtCanTest, SetPositionEncodesExpectedFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x02);
    constexpr int32_t kVelocity = 0x1234;
    constexpr int32_t kPosition = 0x01020304;

    ASSERT_TRUE(mt.setVelocity(kMotorId, kVelocity));
    transport->clearSent();
    ASSERT_TRUE(mt.setPosition(kMotorId, kPosition));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    // MT 位置命令 0xA4：携带速度上限（data[2..3]）和目标位置（data[4..7]）。
    EXPECT_EQ(frame.id, 0x142u);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0xA4);
    EXPECT_EQ(frame.data[2], static_cast<uint8_t>(kVelocity & 0xFF));
    EXPECT_EQ(frame.data[3], static_cast<uint8_t>((kVelocity >> 8) & 0xFF));
    EXPECT_EQ(frame.data[4], static_cast<uint8_t>(kPosition & 0xFF));
    EXPECT_EQ(frame.data[5], static_cast<uint8_t>((kPosition >> 8) & 0xFF));
    EXPECT_EQ(frame.data[6], static_cast<uint8_t>((kPosition >> 16) & 0xFF));
    EXPECT_EQ(frame.data[7], static_cast<uint8_t>((kPosition >> 24) & 0xFF));
}

TEST_F(MtCanTest, SetPositionWithoutVelocityUsesDefaultSpeed)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x02);
    constexpr int32_t kPosition = 0x01020304;

    ASSERT_TRUE(mt.setPosition(kMotorId, kPosition));
    ASSERT_EQ(transport->sentFrames.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    EXPECT_EQ(frame.data[0], 0xA4);
    EXPECT_EQ(frame.data[2], 100u);
    EXPECT_EQ(frame.data[3], 0u);
}

TEST_F(MtCanTest, HandleResponseParsesStateFrame)
{
    // 响应 CAN ID=0x240+nodeId，nodeId=1 -> CAN ID 0x241。
    constexpr MotorID kResponseNodeId = static_cast<MotorID>(0x01);

    // 0x9C 状态反馈帧：电流/速度等状态由协议层解码后更新缓存。
    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0x9C;
    frame.data[2] = 0x7B; // current raw low byte (123)
    frame.data[3] = 0x00; // current raw high byte
    frame.data[4] = 0x58; // velocity raw low byte (600)
    frame.data[5] = 0x02; // velocity raw high byte

    transport->simulateReceive(frame);

    EXPECT_EQ(mt.getCurrent(kResponseNodeId), 123);
    EXPECT_EQ(mt.getVelocity(kResponseNodeId), 100);
}

TEST_F(MtCanTest, HandleResponseIgnoresExtendedFrame)
{
    constexpr MotorID kResponseNodeId = static_cast<MotorID>(0x01);

    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = true;
    frame.data[0] = 0x9C;
    frame.data[2] = 0x7B;
    frame.data[3] = 0x00;
    frame.data[4] = 0x58;
    frame.data[5] = 0x02;

    transport->simulateReceive(frame);

    EXPECT_EQ(mt.getCurrent(kResponseNodeId), 0);
    EXPECT_EQ(mt.getVelocity(kResponseNodeId), 0);
}

TEST_F(MtCanTest, GetPositionPreservesLargeMultiTurnAngle)
{
    constexpr MotorID kResponseNodeId = static_cast<MotorID>(0x01);

    // 0x0000_8000_0000 (48-bit LE) = 2,147,483,648，超过 int32 上界。
    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0x92;
    frame.data[1] = 0x00;
    frame.data[2] = 0x00;
    frame.data[3] = 0x00;
    frame.data[4] = 0x00;
    frame.data[5] = 0x80;
    frame.data[6] = 0x00;
    frame.data[7] = 0x00;

    transport->simulateReceive(frame);

    EXPECT_EQ(mt.getPosition(kResponseNodeId), 2147483648LL);
}

TEST_F(MtCanTest, EnableDisableAndFaultStateAreObservable)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x01);
    EXPECT_TRUE(mt.Enable(kMotorId));
    EXPECT_TRUE(mt.isEnabled(kMotorId));

    EXPECT_TRUE(mt.Disable(kMotorId));
    EXPECT_FALSE(mt.isEnabled(kMotorId));
    ASSERT_FALSE(transport->sentFrames.empty());
    EXPECT_EQ(transport->sentFrames.back().data[0], 0x80);

    CanTransport::Frame frame {};
    frame.id = 0x241;
    frame.dlc = 8;
    frame.isExtended = false;
    frame.isRemoteRequest = false;
    frame.data[0] = 0x9A;
    frame.data[6] = 0x01;
    frame.data[7] = 0x00;
    transport->simulateReceive(frame);
    EXPECT_TRUE(mt.hasFault(kMotorId));
}

TEST_F(MtCanTest, DISABLED_TODO_HandleResponseErrorCodeBranch)
{
    GTEST_SKIP() << "TODO: cover command 0x9A error decode and state transition.";
}

TEST_F(MtCanTest, DISABLED_TODO_HandleResponseResetBranch)
{
    GTEST_SKIP() << "TODO: cover command 0x64 reset flow and emitted frame.";
}
