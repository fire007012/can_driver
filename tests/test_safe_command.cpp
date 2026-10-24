#include <gtest/gtest.h>

#include "can_driver/SafeCommand.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace {
using can_driver::safe_command::clampToInt16;
using can_driver::safe_command::scaleAndClampToInt32;

class SafeCommandFixture : public ::testing::Test {};
} // namespace

// 覆盖 SafeCommand 的关键边界：正常值、溢出、NaN/Inf、非法 scale。

TEST_F(SafeCommandFixture, ScaleAndClampNormal)
{
    int32_t out = 0;
    const bool ok = scaleAndClampToInt32(100.0, 1.0, "joint", out);
    EXPECT_TRUE(ok);
    EXPECT_EQ(out, 100);
}

TEST_F(SafeCommandFixture, ScaleAndClampOverflowUpper)
{
    int32_t out = 0;
    const bool ok = scaleAndClampToInt32(1e18, 1.0, "joint", out);
    EXPECT_TRUE(ok);
    EXPECT_EQ(out, std::numeric_limits<int32_t>::max());
}

TEST_F(SafeCommandFixture, ScaleAndClampOverflowLower)
{
    int32_t out = 0;
    const bool ok = scaleAndClampToInt32(-1e18, 1.0, "joint", out);
    EXPECT_TRUE(ok);
    EXPECT_EQ(out, std::numeric_limits<int32_t>::min());
}

TEST_F(SafeCommandFixture, ScaleAndClampNaN)
{
    int32_t out = 42;
    const bool ok = scaleAndClampToInt32(std::numeric_limits<double>::quiet_NaN(), 1.0, "joint", out);
    EXPECT_FALSE(ok);
}

TEST_F(SafeCommandFixture, ScaleAndClampInfinity)
{
    int32_t out = 42;
    const bool ok = scaleAndClampToInt32(std::numeric_limits<double>::infinity(), 1.0, "joint", out);
    EXPECT_FALSE(ok);
}

TEST_F(SafeCommandFixture, ScaleAndClampZeroScale)
{
    int32_t out = 42;
    const bool ok = scaleAndClampToInt32(1.0, 0.0, "joint", out);
    EXPECT_FALSE(ok);
}

TEST_F(SafeCommandFixture, ScaleAndClampNegativeScale)
{
    int32_t out = 42;
    const bool ok = scaleAndClampToInt32(1.0, -1.0, "joint", out);
    EXPECT_TRUE(ok);
    EXPECT_EQ(out, -1);
}

TEST_F(SafeCommandFixture, ScaleAndClampTinyScaleZeroCmd)
{
    // 0 / tiny_scale 仍应得到可表示的 0，不能误判为失败。
    int32_t out = 42;
    const bool ok = scaleAndClampToInt32(0.0, 1e-300, "joint", out);
    EXPECT_TRUE(ok);
    EXPECT_EQ(out, 0);
}

TEST_F(SafeCommandFixture, ClampToInt16Bounds)
{
    EXPECT_EQ(clampToInt16(40000.0), std::numeric_limits<int16_t>::max());
    EXPECT_EQ(clampToInt16(-40000.0), std::numeric_limits<int16_t>::min());
    EXPECT_EQ(clampToInt16(100.0), 100);
}
