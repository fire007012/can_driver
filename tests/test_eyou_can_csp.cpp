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
        if (transport) {
            transport->send(request.frame);
        }
    }

    std::shared_ptr<MockTransport> transport;
    std::vector<Request> requests;
};

class EyouCanCSPTest : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }

    EyouCanCSPTest()
        : transport(std::make_shared<MockTransport>())
        , txDispatcher(std::make_shared<MockTxDispatcher>(transport))
        , eyou(transport, txDispatcher)
    {
    }

    std::shared_ptr<MockTransport> transport;
    std::shared_ptr<MockTxDispatcher> txDispatcher;
    EyouCan eyou;
};

} // namespace

TEST_F(EyouCanCSPTest, QuickSetPositionFirstCallPreconfiguresVelocityThenWritesPosition)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);
    constexpr int32_t kPosition = 0x01020304;

    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, kPosition));
    ASSERT_EQ(transport->sentFrames.size(), 2u);
    ASSERT_EQ(txDispatcher->requests.size(), 2u);

    const auto &velocityFrame = transport->sentFrames[0];
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

    const auto &positionFrame = transport->sentFrames[1];
    // CSP 模式：预配置速度后，使用 CMD=0x05 快写位置寄存器
    EXPECT_EQ(positionFrame.id, 0x0005u);
    EXPECT_FALSE(positionFrame.isExtended);
    EXPECT_FALSE(positionFrame.isRemoteRequest);
    EXPECT_EQ(positionFrame.dlc, 8u);
    EXPECT_EQ(positionFrame.data[0], 0x05);
    EXPECT_EQ(positionFrame.data[1], 0x0A);
    EXPECT_EQ(positionFrame.data[2], 0x01);
    EXPECT_EQ(positionFrame.data[3], 0x02);
    EXPECT_EQ(positionFrame.data[4], 0x03);
    EXPECT_EQ(positionFrame.data[5], 0x04);
    EXPECT_EQ(positionFrame.data[6], 0x00);
    EXPECT_EQ(positionFrame.data[7], 0x00);
}

TEST_F(EyouCanCSPTest, CSPModeCanBeSet)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    ASSERT_TRUE(eyou.setMode(kMotorId, CanProtocol::MotorMode::CSP));
    ASSERT_EQ(transport->sentFrames.size(), 1u);
    ASSERT_EQ(txDispatcher->requests.size(), 1u);

    const auto &frame = transport->sentFrames[0];
    // 模式设置：CMD=0x01 + 0x0F 模式寄存器 + 模式值 5
    EXPECT_EQ(frame.id, 0x0005u);
    EXPECT_EQ(frame.dlc, 8u);
    EXPECT_EQ(frame.data[0], 0x01); // write command
    EXPECT_EQ(frame.data[1], 0x0F); // mode register
    EXPECT_EQ(frame.data[2], 0x00); // mode value = 5 (big-endian)
    EXPECT_EQ(frame.data[3], 0x00);
    EXPECT_EQ(frame.data[4], 0x00);
    EXPECT_EQ(frame.data[5], 0x05); // CSP mode
}

TEST_F(EyouCanCSPTest, QuickSetPositionSteadyStateOnlySendsPositionFrame)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 1000));
    EXPECT_EQ(transport->sentFrames.size(), 2u);

    transport->clearSent();
    txDispatcher->requests.clear();
    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 2000));
    EXPECT_EQ(transport->sentFrames.size(), 1u);
    EXPECT_EQ(txDispatcher->requests.size(), 1u);
    EXPECT_EQ(transport->sentFrames[0].data[0], 0x05);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x0A);
}

TEST_F(EyouCanCSPTest, QuickSetPositionResendsVelocityWhenDefaultVelocityChanges)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 1000));
    transport->clearSent();
    txDispatcher->requests.clear();

    eyou.setDefaultCspVelocityRaw(0x00001234);

    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 2000));
    ASSERT_EQ(transport->sentFrames.size(), 2u);
    ASSERT_EQ(txDispatcher->requests.size(), 2u);
    EXPECT_EQ(transport->sentFrames[0].data[0], 0x01);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[0].data[4], 0x12);
    EXPECT_EQ(transport->sentFrames[0].data[5], 0x34);
    EXPECT_EQ(transport->sentFrames[1].data[0], 0x05);
    EXPECT_EQ(transport->sentFrames[1].data[1], 0x0A);
}

TEST_F(EyouCanCSPTest, QuickSetPositionUsesPerMotorCspVelocityOverride)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    eyou.setDefaultCspVelocityRaw(0x00001234);
    eyou.setMotorDefaultCspVelocityRaw(kMotorId, 0x00005678);

    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 2000));
    ASSERT_EQ(transport->sentFrames.size(), 2u);
    EXPECT_EQ(transport->sentFrames[0].data[0], 0x01);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[0].data[4], 0x56);
    EXPECT_EQ(transport->sentFrames[0].data[5], 0x78);
    EXPECT_EQ(transport->sentFrames[1].data[0], 0x05);
    EXPECT_EQ(transport->sentFrames[1].data[1], 0x0A);
}

TEST_F(EyouCanCSPTest, QuickSetPositionResendsVelocityAfterModeSwitch)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 1000));
    EXPECT_EQ(transport->sentFrames.size(), 2u);

    transport->clearSent();
    txDispatcher->requests.clear();
    ASSERT_TRUE(eyou.setMode(kMotorId, CanProtocol::MotorMode::Position));
    ASSERT_TRUE(eyou.setMode(kMotorId, CanProtocol::MotorMode::CSP));

    transport->clearSent();
    txDispatcher->requests.clear();
    ASSERT_TRUE(eyou.quickSetPosition(kMotorId, 2000));
    ASSERT_EQ(transport->sentFrames.size(), 2u);
    EXPECT_EQ(transport->sentFrames[0].data[0], 0x01);
    EXPECT_EQ(transport->sentFrames[0].data[1], 0x09);
    EXPECT_EQ(transport->sentFrames[1].data[0], 0x05);
    EXPECT_EQ(transport->sentFrames[1].data[1], 0x0A);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
