#ifndef CAN_DRIVER_JOINT_CONFIG_PARSER_H
#define CAN_DRIVER_JOINT_CONFIG_PARSER_H

#include "can_driver/CanType.h"
#include "can_driver/MotorID.h"

#include <xmlrpcpp/XmlRpcValue.h>

#include <cstdint>
#include <string>
#include <vector>

namespace joint_config_parser {

/**
 * @brief 解析后的单关节配置。
 */
struct ParsedJointConfig {
    std::string name;        ///< 关节名称
    std::string safetyGroup{"default"}; ///< 故障扩散安全组
    std::string canDevice;   ///< 绑定的 CAN 设备名（如 canable0）
    std::string controlMode; ///< 控制模式字符串（position/velocity）
    MotorID motorId{MotorID::LeftWheel}; ///< 电机 ID
    CanType protocol{CanType::MT};       ///< 协议类型
    std::uint32_t dmMasterId{0};         ///< DM 协议反馈使用的 Master ID
    double positionScale{1.0};           ///< 位置单位换算比例
    double velocityScale{1.0};           ///< 速度单位换算比例
    double directionSign{1.0};           ///< 方向修正，仅允许 1 或 -1
    double ppPositionDefaultVelocity{0.0}; ///< PP position 默认速度，关节单位/s
    double ppCspDefaultVelocity{0.0};      ///< PP CSP 默认速度，关节单位/s
    bool hasPpPositionDefaultVelocity{false};
    bool hasPpCspDefaultVelocity{false};
    double ipMaxVelocity{1.0};           ///< IP executor 最大速度（rad/s）
    double ipMaxAcceleration{2.0};       ///< IP executor 最大加速度（rad/s^2）
    double ipMaxJerk{10.0};              ///< IP executor 最大 jerk（rad/s^3）
    double ipGoalTolerance{1e-3};        ///< IP executor 终点容差（rad）
    double commandAlignmentTolerance{0.0}; ///< 模式切换后首帧对齐容差（关节单位）
    std::string ecbIp;                   ///< ECB 固定 IP（可选）
    bool ecbAutoDiscovery{false};        ///< ECB 是否自动发现
    int ecbRefreshMs{20};                ///< ECB 刷新周期（ms）
};

/**
 * @brief 解析 joints 参数数组。
 * @param jointList XmlRpc 数组对象
 * @param out 输出的关节配置列表
 * @param errorMsg 失败时返回可读错误信息
 */
bool parse(const XmlRpc::XmlRpcValue &jointList,
           std::vector<ParsedJointConfig> &out,
           std::string &errorMsg);

/**
 * @brief 将 XML 参数中的 motor_id 转为 MotorID。
 *
 * 支持整型和字符串（十进制/十六进制）两种输入形式。
 */
bool parseMotorId(const XmlRpc::XmlRpcValue &value,
                  const std::string &jointName,
                  MotorID &out,
                  std::string &errorMsg);

} // namespace joint_config_parser

#endif // CAN_DRIVER_JOINT_CONFIG_PARSER_H
