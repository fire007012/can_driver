#include "can_driver/SafeCommand.h"

#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <limits>

namespace can_driver {
namespace safe_command {

int32_t clampToInt32(double value)
{
    // NaN/Inf 视为无效命令，回落到 0 防止向总线下发异常值。
    if (!std::isfinite(value)) {
        return 0;
    }
    const double clamped = std::max(
        static_cast<double>(std::numeric_limits<int32_t>::min()),
        std::min(static_cast<double>(std::numeric_limits<int32_t>::max()), value));
    return static_cast<int32_t>(std::llround(clamped));
}

int16_t clampToInt16(double value)
{
    // 与 int32 版本一致，先过滤非有限值再做边界钳制。
    if (!std::isfinite(value)) {
        return 0;
    }
    const double clamped = std::max(
        static_cast<double>(std::numeric_limits<int16_t>::min()),
        std::min(static_cast<double>(std::numeric_limits<int16_t>::max()), value));
    return static_cast<int16_t>(std::lround(clamped));
}

bool scaleAndClampToInt32(double cmdValue,
                          double scale,
                          const std::string &jointName,
                          int32_t &rawValueOut)
{
    // scale==0 或 NaN/Inf 都会导致不可逆变换，直接拒绝本次下发。
    if (!std::isfinite(cmdValue) || !std::isfinite(scale) || scale == 0.0) {
        ROS_ERROR_THROTTLE(1.0,
                           "[CanDriverHW] Invalid command/scale for joint '%s' (cmd=%g, scale=%g).",
                           jointName.c_str(),
                           cmdValue,
                           scale);
        return false;
    }

    // 将 SI 单位命令转为协议原始计数值（raw = cmd / scale）。
    const double raw = cmdValue / scale;
    if (!std::isfinite(raw)) {
        ROS_ERROR_THROTTLE(1.0,
                           "[CanDriverHW] Non-finite raw command for joint '%s' (cmd=%g, scale=%g).",
                           jointName.c_str(),
                           cmdValue,
                           scale);
        return false;
    }

    rawValueOut = clampToInt32(raw);
    return true;
}

} // namespace safe_command
} // namespace can_driver
