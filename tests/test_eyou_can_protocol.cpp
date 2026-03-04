#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/EyouCan.h"

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
    EXPECT_EQ(frame.id, 0x0005u);
    EXPECT_FALSE(frame.isExtended);
    EXPECT_FALSE(frame.isRemoteRequest);
    EXPECT_EQ(frame.dlc, 6u);
    EXPECT_EQ(frame.data[0], 0x01); // write
    EXPECT_EQ(frame.data[1], 0x0A); // position sub-command
    EXPECT_EQ(frame.data[2], 0x01);
    EXPECT_EQ(frame.data[3], 0x02);
    EXPECT_EQ(frame.data[4], 0x03);
    EXPECT_EQ(frame.data[5], 0x04);
}

TEST_F(EyouCanTest, GetPositionWithoutCacheSendsReadRequest)
{
    constexpr MotorID kMotorId = static_cast<MotorID>(0x05);

    const int32_t pos = eyou.getPosition(kMotorId);

    EXPECT_EQ(pos, 0);
    ASSERT_EQ(transport->sentFrames.size(), 1u);
    const auto &frame = transport->sentFrames[0];
    EXPECT_EQ(frame.id, 0x0005u);
    EXPECT_EQ(frame.dlc, 2u);
    EXPECT_EQ(frame.data[0], 0x03); // read
    EXPECT_EQ(frame.data[1], 0x07); // position register
}

TEST_F(EyouCanTest, HandleReadResponseUpdatesPositionCache)
{
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

TEST_F(EyouCanTest, DISABLED_TODO_HandleWriteAckModeAndEnable)
{
    GTEST_SKIP() << "TODO: cover write ack (0x02) branches for mode/enable.";
}

TEST_F(EyouCanTest, DISABLED_TODO_HandleErrorCodeReadResponse)
{
    GTEST_SKIP() << "TODO: cover read response (0x15) error-report branch.";
}
