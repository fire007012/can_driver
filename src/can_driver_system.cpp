#include "can_driver/can_driver_system.hpp"

#include "can_driver/AxisCommandSemantics.h"
#include "can_driver/SafeCommand.h"
#include "can_driver/ros2_compat.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <charconv>
#include <cctype>
#include <cmath>
#include <map>
#include <set>

namespace can_driver {

namespace {

std::string lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

} // namespace

std::string CanDriverSystem::parameter(const hardware_interface::ComponentInfo &joint,
                                       const std::string &name,
                                       const std::string &fallback)
{
    const auto it = joint.parameters.find(name);
    return it == joint.parameters.end() ? fallback : it->second;
}

bool CanDriverSystem::parseMotorId(const std::string &value, MotorID *out)
{
    if (out == nullptr || value.empty()) {
        return false;
    }
    try {
        std::size_t consumed = 0;
        const int base = value.size() > 2 && value[0] == '0' &&
                                 (value[1] == 'x' || value[1] == 'X')
                             ? 16
                             : 10;
        const auto raw = std::stoul(value, &consumed, base);
        if (consumed != value.size() || raw > 0xffffu) {
            return false;
        }
        *out = static_cast<MotorID>(static_cast<std::uint16_t>(raw));
        return true;
    } catch (...) {
        return false;
    }
}

CanType CanDriverSystem::parseProtocol(const std::string &value)
{
    const auto normalized = lower(value);
    if (normalized == "pp") return CanType::PP;
    if (normalized == "dm") return CanType::DM;
    if (normalized == "ecb") return CanType::ECB;
    return CanType::MT;
}

hardware_interface::CallbackReturn CanDriverSystem::on_init(
    const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    if (info.joints.empty()) {
        ROS_ERROR("[CanDriverSystem] Hardware has no joints.");
        return hardware_interface::CallbackReturn::ERROR;
    }

    axes_.clear();
    stateInterfaceNames_.clear();
    commandInterfaceNames_.clear();
    axes_.reserve(info.joints.size());
    for (std::size_t index = 0; index < info.joints.size(); ++index) {
        const auto &joint = info.joints[index];
        Axis axis;
        axis.config.name = joint.name;
        axis.config.canDevice = parameter(joint, "can_device", "vcan0");
        axis.config.controlMode = parameter(joint, "control_mode", "velocity");
        axis.config.protocol = parseProtocol(parameter(joint, "protocol", "MT"));
        if (!parseMotorId(parameter(joint, "motor_id", std::to_string(index + 1)),
                          &axis.config.motorId)) {
            ROS_ERROR("[CanDriverSystem] Invalid motor_id for joint '%s'.", joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        axis.config.positionScale = std::stod(parameter(joint, "position_scale", "1.0"));
        axis.config.velocityScale = std::stod(parameter(joint, "velocity_scale", "1.0"));
        axis.config.directionSign = std::stod(parameter(joint, "direction_sign", "1.0"));
        axis.config.dmMasterId = static_cast<std::uint32_t>(
            std::stoul(parameter(joint, "master_id", "0"), nullptr, 0));
        if (axis.config.positionScale >= 2.0) {
            axis.config.positionScale = 2.0 * M_PI / axis.config.positionScale;
        }
        if (axis.config.velocityScale >= 2.0) {
            axis.config.velocityScale = 2.0 * M_PI / axis.config.velocityScale;
        }
        if (axis.config.positionScale <= 0.0 || axis.config.velocityScale <= 0.0 ||
            (axis.config.directionSign != 1.0 && axis.config.directionSign != -1.0)) {
            ROS_ERROR("[CanDriverSystem] Invalid scale or direction for joint '%s'.",
                      joint.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        for (const auto &state : joint.state_interfaces) {
            stateInterfaceNames_.push_back(state.name);
        }
        for (const auto &command : joint.command_interfaces) {
            commandInterfaceNames_.push_back(command.name);
        }
        axes_.push_back(std::move(axis));
    }
    deviceManager_ = std::make_shared<DeviceManager>();
    return hardware_interface::CallbackReturn::SUCCESS;
}

bool CanDriverSystem::initDevices()
{
    std::map<std::string, std::vector<std::pair<CanType, MotorID>>> grouped;
    for (const auto &axis : axes_) {
        grouped[axis.config.canDevice].emplace_back(axis.config.protocol, axis.config.motorId);
    }
    for (const auto &entry : grouped) {
        if (!deviceManager_->initDevice(entry.first, entry.second, false)) {
            ROS_ERROR("[CanDriverSystem] Failed to initialize device '%s'.", entry.first.c_str());
            shutdownDevices();
            return false;
        }
    }
    for (const auto &axis : axes_) {
        if (axis.config.protocol == CanType::DM && axis.config.dmMasterId != 0) {
            deviceManager_->configureDmMotorMasterId(axis.config.canDevice,
                                                     axis.config.motorId,
                                                     axis.config.dmMasterId);
        }
        const auto protocol = deviceManager_->getProtocol(axis.config.canDevice,
                                                           axis.config.protocol);
        if (!protocol) return false;
        const auto mode = axisControlModeFromString(axis.config.controlMode);
        const auto target = mode == AxisControlMode::Velocity
                                ? CanProtocol::MotorMode::Velocity
                                : mode == AxisControlMode::Csp
                                    ? CanProtocol::MotorMode::CSP
                                    : CanProtocol::MotorMode::Position;
        if (!protocol->setMode(axis.config.motorId, target)) {
            ROS_ERROR("[CanDriverSystem] Failed to set mode for joint '%s'.", axis.config.name.c_str());
            shutdownDevices();
            return false;
        }
    }
    devicesInitialized_ = true;
    return true;
}

void CanDriverSystem::shutdownDevices()
{
    if (deviceManager_) {
        for (const auto &axis : axes_) {
            const auto protocol = deviceManager_->getProtocol(axis.config.canDevice,
                                                               axis.config.protocol);
            if (protocol) {
                protocol->Stop(axis.config.motorId);
                protocol->Disable(axis.config.motorId);
            }
        }
        deviceManager_->shutdownAll();
    }
    devicesInitialized_ = false;
}

hardware_interface::CallbackReturn CanDriverSystem::on_configure(
    const rclcpp_lifecycle::State &)
{
    return initDevices() ? hardware_interface::CallbackReturn::SUCCESS
                         : hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn CanDriverSystem::on_cleanup(
    const rclcpp_lifecycle::State &)
{
    shutdownDevices();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanDriverSystem::on_activate(
    const rclcpp_lifecycle::State &)
{
    if (!devicesInitialized_) return hardware_interface::CallbackReturn::ERROR;
    for (const auto &axis : axes_) {
        const auto protocol = deviceManager_->getProtocol(axis.config.canDevice,
                                                           axis.config.protocol);
        if (!protocol || !protocol->Enable(axis.config.motorId)) {
            ROS_ERROR("[CanDriverSystem] Failed to enable joint '%s'.", axis.config.name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanDriverSystem::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    if (deviceManager_) {
        for (const auto &axis : axes_) {
            const auto protocol = deviceManager_->getProtocol(axis.config.canDevice,
                                                               axis.config.protocol);
            if (protocol) {
                protocol->Stop(axis.config.motorId);
                protocol->Disable(axis.config.motorId);
            }
        }
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanDriverSystem::on_shutdown(
    const rclcpp_lifecycle::State &)
{
    shutdownDevices();
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CanDriverSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> interfaces;
    for (auto &axis : axes_) {
        interfaces.emplace_back(axis.config.name, hardware_interface::HW_IF_POSITION,
                                &axis.position);
        interfaces.emplace_back(axis.config.name, hardware_interface::HW_IF_VELOCITY,
                                &axis.velocity);
        interfaces.emplace_back(axis.config.name, hardware_interface::HW_IF_EFFORT,
                                &axis.effort);
    }
    return interfaces;
}

std::vector<hardware_interface::CommandInterface> CanDriverSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> interfaces;
    for (auto &axis : axes_) {
        const auto mode = axisControlModeFromString(axis.config.controlMode);
        if (mode == AxisControlMode::Velocity) {
            interfaces.emplace_back(axis.config.name, hardware_interface::HW_IF_VELOCITY,
                                    &axis.velocityCommand);
        } else {
            interfaces.emplace_back(axis.config.name, hardware_interface::HW_IF_POSITION,
                                    &axis.positionCommand);
        }
    }
    return interfaces;
}

hardware_interface::return_type CanDriverSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!devicesInitialized_) return hardware_interface::return_type::ERROR;
    for (auto &axis : axes_) {
        const auto protocol = deviceManager_->getProtocol(axis.config.canDevice,
                                                           axis.config.protocol);
        if (!protocol) return hardware_interface::return_type::ERROR;
        axis.position = rawPositionToJointPosition(axis.config,
                                                    protocol->getPosition(axis.config.motorId));
        axis.velocity = static_cast<double>(protocol->getVelocity(axis.config.motorId)) *
                        effectiveVelocityScale(axis.config);
        axis.effort = static_cast<double>(protocol->getCurrent(axis.config.motorId));
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanDriverSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!devicesInitialized_) return hardware_interface::return_type::ERROR;
    for (auto &axis : axes_) {
        const auto protocol = deviceManager_->getProtocol(axis.config.canDevice,
                                                           axis.config.protocol);
        if (!protocol) return hardware_interface::return_type::ERROR;
        const auto mode = axisControlModeFromString(axis.config.controlMode);
        bool ok = false;
        if (mode == AxisControlMode::Velocity) {
            std::int32_t raw = 0;
            if (!safe_command::scaleAndClampToInt32(axis.velocityCommand,
                                                    effectiveVelocityScale(axis.config),
                                                    axis.config.name,
                                                    raw)) {
                return hardware_interface::return_type::ERROR;
            }
            ok = protocol->setVelocity(axis.config.motorId, raw);
        } else {
            const auto raw = static_cast<std::int32_t>(std::llround(
                jointPositionToRawCount(axis.config, axis.positionCommand)));
            ok = mode == AxisControlMode::Csp
                     ? protocol->quickSetPosition(axis.config.motorId, raw)
                     : protocol->setPosition(axis.config.motorId, raw);
        }
        if (!ok) return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
}

} // namespace can_driver

PLUGINLIB_EXPORT_CLASS(can_driver::CanDriverSystem, hardware_interface::SystemInterface)
