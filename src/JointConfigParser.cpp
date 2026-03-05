#include "can_driver/JointConfigParser.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace joint_config_parser {

// 支持 int 或 string（十进制/十六进制）两种配置形式。
bool parseMotorId(const XmlRpc::XmlRpcValue &value,
                  const std::string &jointName,
                  MotorID &out,
                  std::string &errorMsg)
{
    int rawId = 0;
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        rawId = static_cast<int>(value);
    } else if (value.getType() == XmlRpc::XmlRpcValue::TypeString) {
        try {
            rawId = static_cast<int>(std::stoul(static_cast<std::string>(value), nullptr, 0));
        } catch (const std::exception &e) {
            errorMsg = "Joint '" + jointName + "': invalid motor_id '" +
                       static_cast<std::string>(value) + "' (" + e.what() + ").";
            return false;
        }
    } else {
        errorMsg = "Joint '" + jointName + "': motor_id must be int or string.";
        return false;
    }
    // MotorID 在系统中按 uint16_t 解释，解析时做边界保护。
    if (rawId < 0 || rawId > static_cast<int>(std::numeric_limits<uint16_t>::max())) {
        errorMsg = "Joint '" + jointName + "': motor_id out of range [0, 65535]: " +
                   std::to_string(rawId) + ".";
        return false;
    }
    out = static_cast<MotorID>(static_cast<uint16_t>(rawId));
    return true;
}

bool parse(const XmlRpc::XmlRpcValue &jointList,
           std::vector<ParsedJointConfig> &out,
           std::string &errorMsg)
{
    // joints 必须为非空数组，否则无法构建 hardware_interface 映射。
    if (jointList.getType() != XmlRpc::XmlRpcValue::TypeArray || jointList.size() == 0) {
        errorMsg = "'joints' must be a non-empty list.";
        return false;
    }

    out.clear();
    out.reserve(jointList.size());
    for (int i = 0; i < jointList.size(); ++i) {
        const auto &jv = jointList[i];
        // 必填字段保持严格校验，避免运行期出现“半配置”关节。
        if (!jv.hasMember("name") || !jv.hasMember("motor_id") ||
            !jv.hasMember("protocol") || !jv.hasMember("can_device") ||
            !jv.hasMember("control_mode")) {
            errorMsg = "Joint [" + std::to_string(i) +
                       "] missing required field (name/motor_id/protocol/can_device/control_mode).";
            return false;
        }

        ParsedJointConfig jc;
        jc.name = static_cast<std::string>(jv["name"]);
        jc.canDevice = static_cast<std::string>(jv["can_device"]);
        jc.controlMode = static_cast<std::string>(jv["control_mode"]);
        if (jc.controlMode != "velocity" && jc.controlMode != "position") {
            errorMsg = "Joint '" + jc.name + "': unknown control_mode '" + jc.controlMode +
                       "' (use velocity or position).";
            return false;
        }

        // scale 为可选参数，未配置时使用 1.0（即不缩放）。
        if (jv.hasMember("position_scale")) {
            jc.positionScale = static_cast<double>(jv["position_scale"]);
        }
        if (jv.hasMember("velocity_scale")) {
            jc.velocityScale = static_cast<double>(jv["velocity_scale"]);
        }
        if (!std::isfinite(jc.positionScale) || jc.positionScale <= 0.0) {
            errorMsg = "Joint '" + jc.name + "': invalid position_scale.";
            return false;
        }
        if (!std::isfinite(jc.velocityScale) || jc.velocityScale <= 0.0) {
            errorMsg = "Joint '" + jc.name + "': invalid velocity_scale.";
            return false;
        }

        if (!parseMotorId(jv["motor_id"], jc.name, jc.motorId, errorMsg)) {
            return false;
        }

        // 协议字符串显式映射到枚举，未知值直接报错。
        const std::string protoStr = static_cast<std::string>(jv["protocol"]);
        if (protoStr == "MT") {
            jc.protocol = CanType::MT;
        } else if (protoStr == "PP") {
            jc.protocol = CanType::PP;
        } else {
            errorMsg = "Joint '" + jc.name + "': unknown protocol '" + protoStr + "' (use MT or PP).";
            return false;
        }

        out.push_back(jc);
    }

    return true;
}

} // namespace joint_config_parser
