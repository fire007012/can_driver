#include <gtest/gtest.h>
#include <ros/time.h>

#include "can_driver/SafeCommand.h"

#include <cmath>
#include <limits>

namespace {

using can_driver::safe_command::clampToInt16;
using can_driver::safe_command::clampToInt32;
using can_driver::safe_command::scaleAndClampToInt32;

class RosTimeFixture : public ::testing::Test {
protected:
    static void SetUpTestSuite()
    {
        ros::Time::init();
    }
};

} // namespace

// 这些测试聚焦于 can_driver 数值安全边界逻辑。
// CanDriverHW 的真实 write() 限位流程仍需独立集成测试。

TEST_F(RosTimeFixture, ClampToInt16RejectsNaNAndInfinity)
{
    EXPECT_EQ(clampToInt16(std::numeric_limits<double>::quiet_NaN()), 0);
    EXPECT_EQ(clampToInt16(std::numeric_limits<double>::infinity()), 0);
    EXPECT_EQ(clampToInt16(-std::numeric_limits<double>::infinity()), 0);
}

TEST_F(RosTimeFixture, ClampToInt32RejectsNaNAndInfinity)
{
    EXPECT_EQ(clampToInt32(std::numeric_limits<double>::quiet_NaN()), 0);
    EXPECT_EQ(clampToInt32(std::numeric_limits<double>::infinity()), 0);
    EXPECT_EQ(clampToInt32(-std::numeric_limits<double>::infinity()), 0);
}

TEST_F(RosTimeFixture, ClampToInt16RoundsHalfAwayFromZero)
{
    EXPECT_EQ(clampToInt16(1.5), 2);
    EXPECT_EQ(clampToInt16(-1.5), -2);
}

TEST_F(RosTimeFixture, ClampToInt32RoundsHalfAwayFromZero)
{
    EXPECT_EQ(clampToInt32(2.5), 3);
    EXPECT_EQ(clampToInt32(-2.5), -3);
}

TEST_F(RosTimeFixture, ScaleAndClampCombined)
{
    // 测试缩放后再钳制的完整流程
    // scale 的语义：SI 单位值 / scale = 原始计数值
    // 例如：1.0 rad / 0.001 = 1000 counts
    const double cmdRad = 1.0; // 1 rad
    const double scale = 0.001; // 0.001 rad/count，即 1000 counts/rad
    int32_t raw = 0;

    const bool ok = scaleAndClampToInt32(cmdRad, scale, "test", raw);
    EXPECT_TRUE(ok);

    // 1.0 / 0.001 = 1000
    EXPECT_EQ(raw, 1000);
}

TEST_F(RosTimeFixture, ScaleAndClampOverflowProtection)
{
    // 测试极端值的溢出保护
    const double cmdRad = 1e10; // 极大值
    const double scale = 0.001;
    int32_t raw = 0;

    const bool ok = scaleAndClampToInt32(cmdRad, scale, "test", raw);
    EXPECT_TRUE(ok);

    // 1e10 / 0.001 = 1e13，远超 INT32_MAX，应该被钳制
    EXPECT_EQ(raw, std::numeric_limits<int32_t>::max());
}

TEST_F(RosTimeFixture, ScaleAndClampFailureKeepsOutputUnchanged)
{
    int32_t raw = 12345;
    EXPECT_FALSE(scaleAndClampToInt32(std::numeric_limits<double>::quiet_NaN(), 1.0, "test", raw));
    EXPECT_EQ(raw, 12345);

    raw = -54321;
    EXPECT_FALSE(scaleAndClampToInt32(1.0, 0.0, "test", raw));
    EXPECT_EQ(raw, -54321);
}
