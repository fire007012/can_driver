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
#include <thread>
#include <chrono>
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

    if (info.hardware_parameters.count("safety_domain"))
        safety_ = robot_safety::domain(info.hardware_parameters.at("safety_domain"));
    axes_.clear();
    stateInterfaceNames_.clear();
    commandInterfaceNames_.clear();
    axes_.reserve(info.joints.size());
    for (std::size_t index = 0; index < info.joints.size(); ++index) {
        const auto &joint = info.joints[index];
        Axis axis;
        axis.config.name = joint.name;
        for (const auto &interface : joint.command_interfaces) {
            if (interface.name == "position") {
                if (!interface.min.empty()) axis.lower = std::stod(interface.min);
                if (!interface.max.empty()) axis.upper = std::stod(interface.max);
            }
            if (interface.name == "velocity" && !interface.max.empty()) axis.velocityLimit = std::stod(interface.max);
        }
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
    active_ = false;
    if (safety_) safety_->report(info_.name, false);
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
    for (auto &axis : axes_) {
        axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN();
        axis.claimed = false; axis.commandReceived = false;
        const auto protocol = deviceManager_->getProtocol(axis.config.canDevice,
                                                           axis.config.protocol);
        if (protocol && safety_) {
            protocol->Stop(axis.config.motorId);
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            SharedDriverState::AxisFeedbackState feedback;
            const SharedDriverState::AxisKey key{axis.config.canDevice, axis.config.protocol,
                                                  NormalizeProtocolMotorId(axis.config.motorId)};
            bool fresh = false;
            do {
                fresh = deviceManager_->getSharedDriverState()->getAxisFeedback(key, &feedback) &&
                    feedback.feedbackSeen && feedback.positionValid && feedback.velocityValid &&
                    SharedDriverSteadyNowNs() - feedback.lastRxSteadyNs <= 500000000 &&
                    !(feedback.faultValid && feedback.fault);
                if (fresh) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } while (std::chrono::steady_clock::now() < deadline);
            if (!fresh) { stopAll(); return hardware_interface::CallbackReturn::ERROR; }
            // Prime a safe target before enabling; SI command buffers remain unclaimed/NaN.
            bool seeded = false;
            if (axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Velocity)
                seeded = protocol->setVelocity(axis.config.motorId, 0);
            else if (feedback.position >= INT32_MIN && feedback.position <= INT32_MAX)
                seeded = protocol->quickSetPosition(axis.config.motorId, static_cast<int32_t>(feedback.position));
            if (!seeded) { stopAll(); return hardware_interface::CallbackReturn::ERROR; }
        }
        if (!protocol || !protocol->Enable(axis.config.motorId)) {
            ROS_ERROR("[CanDriverSystem] Failed to enable joint '%s'.", axis.config.name.c_str());
            stopAll();
            return hardware_interface::CallbackReturn::ERROR;
        }
        if (safety_) {
            const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
            bool confirmed = false;
            const auto expected = axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Velocity ?
                CanProtocol::MotorMode::Velocity : axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Csp ?
                CanProtocol::MotorMode::CSP : CanProtocol::MotorMode::Position;
            do {
                SharedDriverState::AxisFeedbackState feedback;
                const SharedDriverState::AxisKey key{axis.config.canDevice, axis.config.protocol,
                    NormalizeProtocolMotorId(axis.config.motorId)};
                confirmed = deviceManager_->getSharedDriverState()->getAxisFeedback(key, &feedback) &&
                    feedback.enabledValid && feedback.enabled && feedback.faultValid && !feedback.fault &&
                    feedback.modeValid && feedback.mode == expected &&
                    SharedDriverSteadyNowNs() - feedback.lastRxSteadyNs <= 500000000;
                if (confirmed) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } while (std::chrono::steady_clock::now() < deadline);
            if (!confirmed) { stopAll(); return hardware_interface::CallbackReturn::ERROR; }
        }
    }
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanDriverSystem::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    active_ = false;
    if (safety_) safety_->report(info_.name, false);
    for (auto &axis : axes_) { axis.claimed = false; axis.commandReceived = false; axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN(); }
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
        if (safety_ && active_) {
            SharedDriverState::AxisFeedbackState feedback;
            const SharedDriverState::AxisKey key{axis.config.canDevice, axis.config.protocol,
                                                  NormalizeProtocolMotorId(axis.config.motorId)};
            const auto now = SharedDriverSteadyNowNs();
            if (!deviceManager_->getSharedDriverState()->getAxisFeedback(key, &feedback) ||
                !feedback.feedbackSeen || !feedback.positionValid || !feedback.velocityValid ||
                !feedback.enabledValid || !feedback.enabled || !feedback.faultValid || feedback.fault ||
                now - feedback.lastRxSteadyNs > 500000000) {
                fail("private CAN feedback stale/fault: " + axis.config.name);
                return hardware_interface::return_type::ERROR;
            }
        }
        axis.position = rawPositionToJointPosition(axis.config,
                                                    protocol->getPosition(axis.config.motorId));
        axis.velocity = static_cast<double>(protocol->getVelocity(axis.config.motorId)) *
                        effectiveVelocityScale(axis.config);
        axis.effort = static_cast<double>(protocol->getCurrent(axis.config.motorId));
    }
    if (safety_) safety_->report(info_.name, active_);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanDriverSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    if (!devicesInitialized_) return hardware_interface::return_type::ERROR;
    if (!active_) return hardware_interface::return_type::OK;
    if (safety_ && !safety_->permits(safetyEpoch_)) {
        if (safety_->state() != robot_safety::Domain::State::Armed && !permissionLost_) {
            stopAll(); permissionLost_ = true;
        }
        for (auto &axis : axes_) axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN();
        return hardware_interface::return_type::OK;
    }
    permissionLost_ = false;
    for (auto &axis : axes_) {
        if (!axis.claimed) continue;
        const bool velocity = axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Velocity;
        const double command = velocity ? axis.velocityCommand : axis.positionCommand;
        if (!std::isfinite(command)) {
            if (std::isnan(command) && !axis.commandReceived) continue;
            fail("non-finite private CAN command"); return hardware_interface::return_type::ERROR;
        }
        axis.commandReceived = true;
        if ((!velocity && (axis.positionCommand < axis.lower || axis.positionCommand > axis.upper)) ||
            (velocity && std::abs(axis.velocityCommand) > axis.velocityLimit)) {
            fail("private CAN command outside SI limits: " + axis.config.name);
            return hardware_interface::return_type::ERROR;
        }
        const auto protocol = deviceManager_->getProtocol(axis.config.canDevice,
                                                           axis.config.protocol);
        if (!protocol) { fail("private CAN protocol missing"); return hardware_interface::return_type::ERROR; }
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
            const double count = jointPositionToRawCount(axis.config, axis.positionCommand);
            if (!std::isfinite(count) || count < INT32_MIN || count > INT32_MAX) {
                fail("private CAN position overflow"); return hardware_interface::return_type::ERROR;
            }
            const auto raw = static_cast<std::int32_t>(std::llround(count));
            ok = mode == AxisControlMode::Csp
                     ? protocol->quickSetPosition(axis.config.motorId, raw)
                     : protocol->setPosition(axis.config.motorId, raw);
        }
        if (!ok) { fail("private CAN write failed: " + axis.config.name); return hardware_interface::return_type::ERROR; }
    }
    return hardware_interface::return_type::OK;
}

void CanDriverSystem::stopAll()
{
    if (!deviceManager_) return;
    for (auto &axis : axes_) {
        axis.claimed = false; axis.commandReceived = false;
        axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN();
        auto protocol = deviceManager_->getProtocol(axis.config.canDevice, axis.config.protocol);
        if (protocol) protocol->Stop(axis.config.motorId);
    }
}
bool CanDriverSystem::fail(const std::string &reason)
{
    if (safety_) { safety_->report(info_.name, false); safety_->fault(reason); }
    stopAll(); return false;
}
hardware_interface::return_type CanDriverSystem::perform_command_mode_switch(
    const std::vector<std::string> &start, const std::vector<std::string> &stop)
{
    for (auto &axis : axes_) {
        const auto interface = axis.config.name + "/" +
          (axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Velocity ? "velocity" : "position");
        if (std::find(stop.begin(), stop.end(), interface) != stop.end()) {
            axis.claimed = false; axis.commandReceived = false;
            auto protocol = deviceManager_->getProtocol(axis.config.canDevice, axis.config.protocol);
            if (protocol) protocol->Stop(axis.config.motorId);
        }
        if (std::find(start.begin(), start.end(), interface) != start.end()) { axis.claimed = true; axis.commandReceived = false; }
        if (std::find(stop.begin(), stop.end(), interface) != stop.end() ||
            std::find(start.begin(), start.end(), interface) != start.end())
            axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN();
    }
    if (safety_) safetyEpoch_ = safety_->epoch();
    permissionLost_ = false;
    return hardware_interface::return_type::OK;
}

} // namespace can_driver

PLUGINLIB_EXPORT_CLASS(can_driver::CanDriverSystem, hardware_interface::SystemInterface)
