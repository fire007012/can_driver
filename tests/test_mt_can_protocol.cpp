#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/MtCan.h"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

namespace {

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

TEST_F(MtCanTest, HandleResponseParsesStateFrame)
{
    constexpr MotorID kResponseNodeId = static_cast<MotorID>(0x41);

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
    constexpr MotorID kResponseNodeId = static_cast<MotorID>(0x41);

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

TEST_F(MtCanTest, DISABLED_TODO_HandleResponseErrorCodeBranch)
{
    GTEST_SKIP() << "TODO: cover command 0x9A error decode and state transition.";
}

TEST_F(MtCanTest, DISABLED_TODO_HandleResponseResetBranch)
{
    GTEST_SKIP() << "TODO: cover command 0x64 reset flow and emitted frame.";
}
