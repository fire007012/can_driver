#ifndef CAN_DRIVER_SAFE_COMMAND_H
#define CAN_DRIVER_SAFE_COMMAND_H

#include <cstdint>
#include <string>

namespace can_driver {
namespace safe_command {

/// 将 double 安全转换为 int32_t，超界时钳制，非有限值返回 0。
int32_t clampToInt32(double value);
/// 将 double 安全转换为 int16_t，超界时钳制，非有限值返回 0。
int16_t clampToInt16(double value);
/**
 * @brief 先按比例缩放命令值，再安全转换为 int32。
 * @note scale 允许为正或负，但不能为 0；负号可用于方向翻转。
 * @return true 表示输出有效；false 表示输入非法（out 保持不变）
 */
bool scaleAndClampToInt32(double cmdValue,
                          double scale,
                          const std::string &jointName,
                          int32_t &rawValueOut);

} // namespace safe_command
} // namespace can_driver

#endif // CAN_DRIVER_SAFE_COMMAND_H
