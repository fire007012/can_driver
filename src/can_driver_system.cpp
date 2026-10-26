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
    grouped_ = info.hardware_parameters.count("group_policy") != 0;
    if (grouped_) {
        try {
            safety_ = robot_safety::groupedDomain(info.hardware_parameters.at("safety_domain"),
                info.hardware_parameters.at("group_policy"));
            guard_ = robot_safety::guard(info.hardware_parameters.at("safety_domain"), safety_);
        } catch (const std::exception & e) {
            ROS_ERROR("[CanDriverSystem] Group policy rejected: %s", e.what());
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    if (safety_) safety_->registerWriter(info.name, grouped_);
    axes_.clear();
    stateInterfaceNames_.clear();
    commandInterfaceNames_.clear();
    axes_.reserve(info.joints.size());
    for (std::size_t index = 0; index < info.joints.size(); ++index) {
        const auto &joint = info.joints[index];
        Axis axis;
        axis.config.name = joint.name;
        if (grouped_) {
            axis.session.group = safety_->axisGroup(joint.name, info.name);
            axis.feedbackLimits = {
              robot_safety::millisecondsParameter(joint.parameters,"feedback_warn_ms",250)*1000000,
              robot_safety::millisecondsParameter(joint.parameters,"feedback_stop_ms",500)*1000000,
              robot_safety::millisecondsParameter(joint.parameters,"feedback_stable_ms",100)*1000000};
            const std::array<std::string,5> types{"position","velocity","mode","enabled","fault"};
            for (std::size_t i=0;i<types.size();++i) {
                axis.fieldLimits[i] = {
                  robot_safety::millisecondsParameter(joint.parameters,types[i]+"_feedback_warn_ms",axis.feedbackLimits.warn_ns/1000000)*1000000,
                  robot_safety::millisecondsParameter(joint.parameters,types[i]+"_feedback_stop_ms",axis.feedbackLimits.stop_ns/1000000)*1000000,
                  robot_safety::millisecondsParameter(joint.parameters,types[i]+"_feedback_stable_ms",axis.feedbackLimits.stable_ns/1000000)*1000000};
                axis.fieldHealth.emplace(i,robot_safety::Freshness(axis.fieldLimits[i]));
            }
        }
        for (const auto &interface : joint.command_interfaces) {
            if (interface.name == "position") {
                if (!interface.min.empty()) axis.lower = std::stod(interface.min);
                if (!interface.max.empty()) axis.upper = std::stod(interface.max);
            }
            if (interface.name == "velocity" && !interface.max.empty()) axis.velocityLimit = std::stod(interface.max);
        }
        axis.config.canDevice = parameter(joint, "can_device", "vcan0");
        axis.config.controlMode = parameter(joint, "control_mode", "velocity");
        const auto protocolName = lower(parameter(joint,"protocol","MT"));
        if (!std::set<std::string>{"pp","dm","mt","ecb"}.count(protocolName) ||
            axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Unknown)
            return hardware_interface::CallbackReturn::ERROR;
        axis.config.protocol = parseProtocol(protocolName);
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
        if (!std::isfinite(axis.config.positionScale) || !std::isfinite(axis.config.velocityScale) ||
            axis.config.positionScale <= 0.0 || axis.config.velocityScale <= 0.0 ||
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
        if (std::isnan(axis.lower) || std::isnan(axis.upper) || axis.lower >= axis.upper ||
            std::isnan(axis.velocityLimit) || axis.velocityLimit <= 0) return hardware_interface::CallbackReturn::ERROR;
        axes_.push_back(std::move(axis));
    }
    if (!deviceManager_) deviceManager_ = std::make_shared<DeviceManager>();
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
            if (!grouped_) { shutdownDevices(); return false; }
        }
    }
    for (auto &axis : axes_) {
        if (axis.config.protocol == CanType::DM && axis.config.dmMasterId != 0) {
            deviceManager_->configureDmMotorMasterId(axis.config.canDevice,
                                                     axis.config.motorId,
                                                     axis.config.dmMasterId);
        }
        const auto protocol = deviceManager_->getProtocol(axis.config.canDevice,
                                                           axis.config.protocol);
        if (grouped_) { axis.configured = static_cast<bool>(protocol); continue; }
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
    if (grouped_) for (const auto & axis : axes_) {
        const SharedDriverState::AxisKey key{axis.config.canDevice,axis.config.protocol,NormalizeProtocolMotorId(axis.config.motorId)};
        deviceManager_->getSharedDriverState()->mutateAxisFeedback(key,[](auto state) {
            state->positionValid=state->velocityValid=state->currentValid=state->modeValid=false;
            state->enabledValid=state->faultValid=state->feedbackSeen=false;
            state->feedbackSteadyNs.fill(0);state->lastRxSteadyNs=state->lastValidStateSteadyNs=0;
        });
    }
    devicesInitialized_ = true;
    return true;
}

void CanDriverSystem::shutdownDevices()
{
    active_ = false;
    backendServices_.reset();
    if (safety_) {
        if (grouped_) { for (auto & axis : axes_) { axis.session.prepared->store(false);
            safety_->unprepareAxis(axis.config.name, "component inactive"); } }
        else safety_->report(info_.name, false);
    }
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
    for (auto & axis : axes_) axis.position=axis.velocity=axis.effort=std::numeric_limits<double>::quiet_NaN();
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
    for (auto & axis : axes_) {
        axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN();
        axis.claimed = false; axis.commandReceived = false; axis.session.permission_lost = true;
    }
    if (grouped_) {
        active_ = true;
        backendServices_ = std::make_shared<robot_safety::Runtime>("private_can_group_operations");
        std::set<std::string> groups;
        for (const auto & axis : axes_) groups.insert(axis.session.group);
        for (const auto & group : groups) {
            backendServices_->service("/robot_backend/" + info_.name + "/" + group + "/prepare",
                [this, group] { prepareGroup(group); }, group);
        }
        backendServices_->start();
        return hardware_interface::CallbackReturn::SUCCESS;
    }
    for (std::size_t i = 0; i < axes_.size(); ++i) {
        if (!prepareAxis(i)) { stopAll(); return hardware_interface::CallbackReturn::ERROR; }
    }
    active_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CanDriverSystem::on_deactivate(
    const rclcpp_lifecycle::State &)
{
    active_ = false;
    backendServices_.reset();
    if (safety_) {
        if (grouped_) { for (auto & axis : axes_) { axis.session.prepared->store(false);
            safety_->unprepareAxis(axis.config.name, "component inactive"); } }
        else safety_->report(info_.name, false);
    }
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
    if (grouped_) {
        std::set<std::string> channels, failedChannels;
        for (const auto & axis : axes_) channels.insert(axis.config.canDevice);
        for (const auto & channel : channels) {
            SharedDriverState::DeviceHealthState health;
            const bool ready = deviceManager_->isDeviceReady(channel);
            const bool known = deviceManager_->getSharedDriverState()->getDeviceHealth(channel,&health);
            const bool newError = known && health.txError > transportErrors_[channel];
            if (known) transportErrors_[channel] = health.txError;
            if (!ready || !known || !health.transportReady || newError) {
                failedChannels.insert(channel);
                for (const auto & axis : axes_) if (axis.config.canDevice == channel)
                    safety_->reportAxis(axis.config.name,false,"private CAN shared transport unavailable/error");
            }
        }
        for (auto & axis : axes_) {
            if (failedChannels.count(axis.config.canDevice)) {
                axis.position = axis.velocity = axis.effort = std::numeric_limits<double>::quiet_NaN(); continue;
            }
            SharedDriverState::AxisFeedbackState feedback;
            const SharedDriverState::AxisKey key{axis.config.canDevice, axis.config.protocol,
                NormalizeProtocolMotorId(axis.config.motorId)};
            const auto now = SharedDriverSteadyNowNs();
            const bool sample = deviceManager_->getSharedDriverState()->getAxisFeedback(key, &feedback) &&
                feedback.feedbackSeen && feedback.positionValid && feedback.velocityValid &&
                std::isfinite(rawPositionToJointPosition(axis.config,feedback.position)) &&
                std::isfinite(feedback.velocity * effectiveVelocityScale(axis.config));
            const bool valid = sample && feedback.feedbackSteadyNs[0] > 0 && feedback.feedbackSteadyNs[1] > 0 &&
                feedback.feedbackSteadyNs[0] <= now && feedback.feedbackSteadyNs[1] <= now &&
                now-feedback.feedbackSteadyNs[0] < axis.fieldLimits[0].stop_ns &&
                now-feedback.feedbackSteadyNs[1] < axis.fieldLimits[1].stop_ns;
            if (valid) {
                axis.position = rawPositionToJointPosition(axis.config, feedback.position);
                axis.velocity = feedback.velocity * effectiveVelocityScale(axis.config);
                axis.effort = feedback.currentValid ? static_cast<double>(feedback.current) : std::numeric_limits<double>::quiet_NaN();
            } else axis.position = axis.velocity = axis.effort = std::numeric_limits<double>::quiet_NaN();
            if (!axis.session.prepared->load()) {
                safety_->unprepareAxis(axis.config.name, valid ? "not prepared" : "feedback unreadable; not prepared");
                continue;
            }
            const auto expected = axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Velocity ?
                CanProtocol::MotorMode::Velocity : axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Csp ?
                CanProtocol::MotorMode::CSP : CanProtocol::MotorMode::Position;
            const auto prepared = axis.session.prepared_epoch->load();
            if (prepared != axis.observedPreparation) {
                axis.fieldHealth.clear();
                for (std::size_t i=0;i<axis.fieldLimits.size();++i)
                    axis.fieldHealth.emplace(i,robot_safety::Freshness(axis.fieldLimits[i]));
                axis.observedPreparation = prepared;
            }
            const bool hard = !feedback.enabledValid || !feedback.enabled || !feedback.faultValid || feedback.fault ||
                !feedback.modeValid || feedback.mode != expected;
            bool ready=true,warning=false,stabilizing=false,stopped=false;
            std::string failed;
            const std::array<std::string,5> types{"position","velocity","mode","enabled","fault"};
            for (std::size_t i=0;i<axis.fieldLimits.size();++i) {
                auto & monitor=axis.fieldHealth.at(i);
                const auto state=monitor.observe(now,feedback.feedbackSteadyNs[i],sample,hard);
                ready=ready&&monitor.trustworthyForMotion();warning=warning||state==robot_safety::Freshness::State::Warn;
                stabilizing=stabilizing||state==robot_safety::Freshness::State::Stabilizing;
                if (state==robot_safety::Freshness::State::Stop) {stopped=true;if(failed.empty())failed=types[i];}
            }
            safety_->warning(axis.config.name,warning,"private feedback warning; within configured field stop deadlines");
            if (stabilizing&&!stopped) safety_->unprepareAxis(axis.config.name,"waiting for stable private feedback");
            else safety_->reportAxis(axis.config.name,ready,hard?"unexpected private enable/mode/fault":
                stopped?"private "+failed+" feedback deadline exceeded":warning?"private feedback warning":"private feedback valid");
        }
        return hardware_interface::return_type::OK;
    }
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
    if (safety_ && !grouped_ && !safety_->permits(safetyEpoch_)) {
        if (safety_->state() != robot_safety::Domain::State::Armed && !permissionLost_) {
            stopAll(); permissionLost_ = true;
        }
        for (auto &axis : axes_) axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN();
        return hardware_interface::return_type::OK;
    }
    permissionLost_ = false;
    if (grouped_) {
        // Validate every claimed output before emitting any member of the group.
        for (auto & axis : axes_) {
            if (!axis.claimed || !safety_->permits(axis.session.group, axis.session.epoch)) continue;
            const bool velocity = axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Velocity;
            const auto command = velocity ? axis.velocityCommand : axis.positionCommand;
            if (std::isnan(command) && !axis.commandReceived) continue;
            const auto raw = velocity ? command / effectiveVelocityScale(axis.config) : jointPositionToRawCount(axis.config, command);
            if (!std::isfinite(command) || !std::isfinite(raw) || raw < INT32_MIN || raw > INT32_MAX ||
                (velocity ? std::abs(command) > axis.velocityLimit : command < axis.lower || command > axis.upper))
                robot_safety::commandFault(safety_, axis.config.name, axis.session.group, "invalid private CAN output");
        }
    }
    for (auto &axis : axes_) {
        if (grouped_ && !safety_->permits(axis.session.group, axis.session.epoch)) {
            const auto state = safety_->group(axis.session.group).state;
            if ((state != robot_safety::Domain::State::Armed && !axis.session.permission_lost) ||
                (axis.session.prepared->load() && (safety_->globallyInhibited() ||
                 (state != robot_safety::Domain::State::Armed && axis.session.prepared_epoch->load() != safety_->epoch(axis.session.group)))))
                stopGroup(axis.session.group);
            axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN();
            continue;
        }
        axis.session.permission_lost = false;
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
        if (!protocol) {
            if (grouped_) { safety_->reportAxis(axis.config.name, false, "protocol unavailable"); stopGroup(axis.session.group); continue; }
            fail("private CAN protocol missing"); return hardware_interface::return_type::ERROR;
        }
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
        if (!ok) {
            if (grouped_) {
                // A send failure is a shared channel failure, not evidence of a single drive fault.
                for (auto & affected : axes_) if (affected.config.canDevice == axis.config.canDevice)
                    safety_->reportAxis(affected.config.name, false, "private CAN transport write failed");
                for (auto & affected : axes_) if (!safety_->permits(affected.session.group, affected.session.epoch))
                    stopGroup(affected.session.group);
                continue;
            }
            fail("private CAN write failed: " + axis.config.name); return hardware_interface::return_type::ERROR;
        }
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
            if (grouped_) { axis.session.permission_lost=true;axis.session.prepared->store(false); }
            auto protocol = deviceManager_->getProtocol(axis.config.canDevice, axis.config.protocol);
            if (protocol) protocol->Stop(axis.config.motorId);
        }
        if (std::find(start.begin(), start.end(), interface) != start.end()) {
            if (grouped_) {
                if (!axis.session.prepared->load() || safety_->group(axis.session.group).state != robot_safety::Domain::State::Armed)
                    return hardware_interface::return_type::ERROR;
                axis.session.epoch = safety_->epoch(axis.session.group);
            }
            axis.claimed = true; axis.commandReceived = false; axis.session.permission_lost = false;
        }
        if (std::find(stop.begin(), stop.end(), interface) != stop.end() ||
            std::find(start.begin(), start.end(), interface) != start.end())
            axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN();
    }
    if (safety_) safetyEpoch_ = safety_->epoch();
    permissionLost_ = false;
    return hardware_interface::return_type::OK;
}

bool CanDriverSystem::prepareAxis(std::size_t index)
{
    auto & axis = axes_.at(index);
    const auto freshnessLimit = grouped_ ? axis.feedbackLimits.stop_ns : 500000000;
    if (grouped_) {
        const auto protocol = deviceManager_->getProtocol(axis.config.canDevice, axis.config.protocol);
        if (!protocol || !deviceManager_->isDeviceReady(axis.config.canDevice) || !protocol->Stop(axis.config.motorId)) return false;
        const auto mode = axisControlModeFromString(axis.config.controlMode);
        const auto target = mode == AxisControlMode::Velocity ? CanProtocol::MotorMode::Velocity :
            mode == AxisControlMode::Csp ? CanProtocol::MotorMode::CSP : CanProtocol::MotorMode::Position;
        if (!protocol->setMode(axis.config.motorId, target)) return false;
    }
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
                    SharedDriverSteadyNowNs() - (grouped_ ? std::min(feedback.feedbackSteadyNs[0],feedback.feedbackSteadyNs[1]) : feedback.lastRxSteadyNs) < freshnessLimit &&
                    (!grouped_ || (feedback.feedbackSteadyNs[0]>0 && feedback.feedbackSteadyNs[1]>0)) &&
                    !(feedback.faultValid && feedback.fault);
                if (fresh) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } while (std::chrono::steady_clock::now() < deadline);
            if (!fresh) { return false; }
            // Prime a safe target before enabling; SI command buffers remain unclaimed/NaN.
            bool seeded = false;
            if (axisControlModeFromString(axis.config.controlMode) == AxisControlMode::Velocity)
                seeded = protocol->setVelocity(axis.config.motorId, 0);
            else if (feedback.position >= INT32_MIN && feedback.position <= INT32_MAX)
                seeded = protocol->quickSetPosition(axis.config.motorId, static_cast<int32_t>(feedback.position));
            if (!seeded) { return false; }
        }
        if (!protocol || !protocol->Enable(axis.config.motorId)) {
            ROS_ERROR("[CanDriverSystem] Failed to enable joint '%s'.", axis.config.name.c_str());
            return false;
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
                    SharedDriverSteadyNowNs() - (grouped_ ? *std::min_element(feedback.feedbackSteadyNs.begin(),feedback.feedbackSteadyNs.begin()+5) : feedback.lastRxSteadyNs) < freshnessLimit &&
                    (!grouped_ || *std::min_element(feedback.feedbackSteadyNs.begin(),feedback.feedbackSteadyNs.begin()+5)>0);
                if (confirmed) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } while (std::chrono::steady_clock::now() < deadline);
            if (!confirmed) { return false; }
        }
    return true;
}
void CanDriverSystem::prepareGroup(const std::string & group)
{
    if (!active_) throw std::runtime_error("hardware component is not active");
    const auto state = safety_->group(group).state;
    if (state == robot_safety::Domain::State::Running || state == robot_safety::Domain::State::Armed)
      throw std::runtime_error("halt group before device preparation");
    const auto ticket = safety_->epoch(group);
    for (std::size_t i = 0; i < axes_.size(); ++i) {
        auto & axis = axes_[i];
        if (axis.session.group != group) continue;
        axis.session.prepared->store(false);
        if (!active_ || safety_->epoch(group) != ticket || safety_->globallyInhibited())
            throw std::runtime_error("preparation cancelled");
        if (!prepareAxis(i) || !deviceManager_->isDeviceReady(axis.config.canDevice) || !active_ || safety_->epoch(group) != ticket || safety_->globallyInhibited()) {
            auto protocol = deviceManager_->getProtocol(axis.config.canDevice, axis.config.protocol);
            if (protocol) { protocol->Stop(axis.config.motorId); protocol->Disable(axis.config.motorId); }
            safety_->reportAxis(axis.config.name, false, "axis preparation failed/cancelled");
            throw std::runtime_error(axis.config.name + ": preparation failed/cancelled");
        }
        axis.session.prepared_epoch->store(ticket);
        axis.session.prepared->store(true);
    }
}

void CanDriverSystem::stopGroup(const std::string & group)
{
    for (auto & axis : axes_) if (axis.session.group == group) {
        axis.claimed = false; axis.commandReceived = false;
        axis.session.permission_lost = true; axis.session.prepared->store(false);
        axis.positionCommand = axis.velocityCommand = std::numeric_limits<double>::quiet_NaN();
        const auto protocol = deviceManager_->getProtocol(axis.config.canDevice, axis.config.protocol);
        if (protocol && !protocol->Stop(axis.config.motorId))
            ROS_ERROR("[CanDriverSystem] Stop unconfirmed for '%s'", axis.config.name.c_str());
    }
}
} // namespace can_driver

PLUGINLIB_EXPORT_CLASS(can_driver::CanDriverSystem, hardware_interface::SystemInterface)
