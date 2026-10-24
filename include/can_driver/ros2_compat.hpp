#ifndef CAN_DRIVER_ROS2_COMPAT_HPP
#define CAN_DRIVER_ROS2_COMPAT_HPP

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <sstream>

namespace can_driver {

using SteadyTimePoint = std::chrono::steady_clock::time_point;

inline SteadyTimePoint steadyNow()
{
    return std::chrono::steady_clock::now();
}

inline void logf(const char *level, const char *format, ...)
{
    std::fprintf(stderr, "[%s] ", level);
    va_list args;
    va_start(args, format);
    std::vfprintf(stderr, format, args);
    va_end(args);
    std::fputc('\n', stderr);
}

} // namespace can_driver

// The core is shared by the ROS2 node and hardware plugin. These compatibility
// macros keep low-level protocol code independent from either ROS logging API.
#define ROS_INFO(...) ::can_driver::logf("INFO", __VA_ARGS__)
#define ROS_WARN(...) ::can_driver::logf("WARN", __VA_ARGS__)
#define ROS_ERROR(...) ::can_driver::logf("ERROR", __VA_ARGS__)
#define ROS_FATAL(...) ::can_driver::logf("FATAL", __VA_ARGS__)
#define ROS_INFO_STREAM(expr) do { std::ostringstream _can_driver_log; _can_driver_log << expr; std::cerr << "[INFO] " << _can_driver_log.str() << std::endl; } while (false)
#define ROS_WARN_STREAM(expr) do { std::ostringstream _can_driver_log; _can_driver_log << expr; std::cerr << "[WARN] " << _can_driver_log.str() << std::endl; } while (false)
#define ROS_ERROR_STREAM(expr) do { std::ostringstream _can_driver_log; _can_driver_log << expr; std::cerr << "[ERROR] " << _can_driver_log.str() << std::endl; } while (false)
#define ROS_INFO_STREAM_THROTTLE(period, expr) ROS_INFO_STREAM(expr)
#define ROS_WARN_STREAM_THROTTLE(period, expr) ROS_WARN_STREAM(expr)
#define ROS_ERROR_STREAM_THROTTLE(period, expr) ROS_ERROR_STREAM(expr)
#define ROS_WARN_THROTTLE(period, ...) ROS_WARN(__VA_ARGS__)
#define ROS_ERROR_THROTTLE(period, ...) ROS_ERROR(__VA_ARGS__)
#define ROS_WARN_STREAM_COND(condition, expr) \
    do { if (condition) { ROS_WARN_STREAM(expr); } } while (false)

#endif // CAN_DRIVER_ROS2_COMPAT_HPP
