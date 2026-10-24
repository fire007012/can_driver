#include "can_driver/AxisCommandSemantics.h"
#include "can_driver/DeviceManager.h"
#include "can_driver/SafeCommand.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <can_driver/msg/motor_state.hpp>
#include <can_driver/srv/apply_limits.hpp>
#include <can_driver/srv/init.hpp>
#include <can_driver/srv/motor_command.hpp>
#include <can_driver/srv/recover.hpp>
#include <can_driver/srv/set_zero.hpp>
#include <can_driver/srv/set_zero_limit.hpp>
#include <can_driver/srv/shutdown.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

namespace {

using can_driver::CanDriverJointConfig;
using ::CanType;
using ::MotorID;
using MotorState = can_driver::msg::MotorState;

std::string yamlString(const YAML::Node &node, const std::string &key,
                       const std::string &fallback = {})
{
    if (!node[key] || !node[key].IsScalar()) return fallback;
    return node[key].as<std::string>();
}

bool parseMotorId(const YAML::Node &value, MotorID *out)
{
    if (!value || out == nullptr) return false;
    try {
        const auto text = value.as<std::string>();
        std::size_t consumed = 0;
        const int base = text.size() > 2 && text[0] == '0' &&
                                 (text[1] == 'x' || text[1] == 'X') ? 16 : 10;
        const auto raw = std::stoul(text, &consumed, base);
        if (consumed != text.size() || raw > 0xffffu) return false;
        *out = static_cast<MotorID>(static_cast<std::uint16_t>(raw));
        return true;
    } catch (...) {
        return false;
    }
}

CanType parseProtocol(const std::string &value)
{
    if (value == "PP" || value == "pp") return CanType::PP;
    if (value == "DM" || value == "dm") return CanType::DM;
    if (value == "ECB" || value == "ecb") return CanType::ECB;
    return CanType::MT;
}

uint8_t msgMode(CanProtocol::MotorMode mode)
{
    switch (mode) {
    case CanProtocol::MotorMode::Velocity: return MotorState::MODE_VELOCITY;
    case CanProtocol::MotorMode::CSP: return MotorState::MODE_CSP;
    case CanProtocol::MotorMode::Position: return MotorState::MODE_POSITION;
    }
    return MotorState::MODE_UNKNOWN;
}

class CanDriverNode final : public rclcpp::Node {
public:
    CanDriverNode()
        : rclcpp::Node("can_driver_node")
        , deviceManager_(std::make_shared<DeviceManager>())
    {
        const auto defaultConfig = std::filesystem::path(
            ament_index_cpp::get_package_share_directory("can_driver")) /
            "config/can_driver.yaml";
        configFile_ = declare_parameter<std::string>("config_file", defaultConfig.string());
        controlFrequency_ = declare_parameter<double>("control_frequency", 100.0);
        statePeriodSec_ = declare_parameter<double>("motor_state_period_sec", 0.1);
        directTimeoutSec_ = declare_parameter<double>("direct_cmd_timeout_sec", 0.5);
        debugBypass_ = declare_parameter<bool>("debug_bypass_ros_control", false);
        motorQueryHz_ = declare_parameter<double>("motor_query_hz", 0.0);
        ppFastWrite_ = declare_parameter<bool>("pp_fast_write_enabled", false);
        if (!loadConfig(configFile_)) throw std::runtime_error("failed to load configuration");
        createEndpoints();
        std_msgs::msg::String initialState;
        initialState.data = state_;
        lifecycleStatePub_->publish(initialState);
        controlTimer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / std::max(1.0, controlFrequency_))),
            std::bind(&CanDriverNode::controlLoop, this));
        stateTimer_ = create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(std::max(0.01, statePeriodSec_))),
            std::bind(&CanDriverNode::publishMotorStates, this));
        RCLCPP_INFO(get_logger(), "Loaded %zu joints from %s", axes_.size(), configFile_.c_str());
    }

    ~CanDriverNode() override { shutdownDevices(); }

private:
    struct Axis {
        CanDriverJointConfig config;
        bool hasDirect{false};
        double directValue{0.0};
        std::chrono::steady_clock::time_point lastCommand{};
        bool stopIssuedOnFault{false};
    };

    bool loadConfig(const std::string &path)
    {
        YAML::Node root;
        try { root = YAML::LoadFile(path); }
        catch (const std::exception &error) {
            RCLCPP_ERROR(get_logger(), "Cannot load config '%s': %s", path.c_str(), error.what());
            return false;
        }
        YAML::Node node = root["can_driver_node"] ? root["can_driver_node"] : root;
        if (node["ros__parameters"]) node = node["ros__parameters"];
        if (node["control_frequency"]) controlFrequency_ = node["control_frequency"].as<double>();
        if (node["motor_state_period_sec"]) statePeriodSec_ = node["motor_state_period_sec"].as<double>();
        if (node["direct_cmd_timeout_sec"]) directTimeoutSec_ = node["direct_cmd_timeout_sec"].as<double>();
        if (node["motor_query_hz"]) motorQueryHz_ = node["motor_query_hz"].as<double>();
        if (node["debug_bypass_ros_control"]) debugBypass_ = node["debug_bypass_ros_control"].as<bool>();
        if (node["pp_fast_write_enabled"]) ppFastWrite_ = node["pp_fast_write_enabled"].as<bool>();
        const auto joints = node["joints"];
        if (!joints || !joints.IsSequence() || joints.size() == 0) {
            RCLCPP_ERROR(get_logger(), "Configuration must contain a non-empty joints sequence.");
            return false;
        }
        for (const auto &entry : joints) {
            Axis axis;
            axis.config.name = entry["name"].as<std::string>();
            if (!parseMotorId(entry["motor_id"], &axis.config.motorId)) {
                RCLCPP_ERROR(get_logger(), "Invalid motor_id for joint '%s'.", axis.config.name.c_str());
                return false;
            }
            axis.config.protocol = parseProtocol(yamlString(entry, "protocol", "MT"));
            axis.config.canDevice = yamlString(entry, "can_device", "vcan0");
            axis.config.controlMode = yamlString(entry, "control_mode", "velocity");
            axis.config.positionScale = entry["position_scale"] ? entry["position_scale"].as<double>() : 1.0;
            axis.config.velocityScale = entry["velocity_scale"] ? entry["velocity_scale"].as<double>() : 1.0;
            axis.config.directionSign = entry["direction_sign"] ? entry["direction_sign"].as<double>() : 1.0;
            axis.config.dmMasterId = entry["master_id"] ? entry["master_id"].as<std::uint32_t>() : 0;
            if (axis.config.positionScale >= 2.0) axis.config.positionScale = 2.0 * M_PI / axis.config.positionScale;
            if (axis.config.velocityScale >= 2.0) axis.config.velocityScale = 2.0 * M_PI / axis.config.velocityScale;
            if (axis.config.positionScale <= 0.0 || axis.config.velocityScale <= 0.0 ||
                (axis.config.directionSign != 1.0 && axis.config.directionSign != -1.0)) {
                RCLCPP_ERROR(get_logger(), "Invalid scale/direction for joint '%s'.", axis.config.name.c_str());
                return false;
            }
            axes_.push_back(std::move(axis));
        }
        return true;
    }

    void createEndpoints()
    {
        motorStatesPub_ = create_publisher<MotorState>("motor_states", rclcpp::QoS(10).reliable());
        lifecycleStatePub_ = create_publisher<std_msgs::msg::String>(
            "lifecycle_state", rclcpp::QoS(1).reliable().transient_local());
        for (std::size_t i = 0; i < axes_.size(); ++i) {
            const auto name = axes_[i].config.name;
            if (can_driver::controlModeUsesVelocitySemantics(axes_[i].config.controlMode)) {
                velSubs_[name] = create_subscription<std_msgs::msg::Float64>(
                    "motor/" + name + "/cmd_velocity", rclcpp::QoS(1),
                    [this, i](std_msgs::msg::Float64::ConstSharedPtr message) { acceptDirect(i, message->data); });
            } else {
                posSubs_[name] = create_subscription<std_msgs::msg::Float64>(
                    "motor/" + name + "/cmd_position", rclcpp::QoS(1),
                    [this, i](std_msgs::msg::Float64::ConstSharedPtr message) { acceptDirect(i, message->data); });
            }
        }
        initSrv_ = create_service<can_driver::srv::Init>("init", std::bind(&CanDriverNode::onInit, this, std::placeholders::_1, std::placeholders::_2));
        shutdownSrv_ = create_service<can_driver::srv::Shutdown>("shutdown", std::bind(&CanDriverNode::onShutdown, this, std::placeholders::_1, std::placeholders::_2));
        recoverSrv_ = create_service<can_driver::srv::Recover>("recover", std::bind(&CanDriverNode::onRecover, this, std::placeholders::_1, std::placeholders::_2));
        motorCmdSrv_ = create_service<can_driver::srv::MotorCommand>("motor_command", std::bind(&CanDriverNode::onMotorCommand, this, std::placeholders::_1, std::placeholders::_2));
        setZeroSrv_ = create_service<can_driver::srv::SetZero>("set_zero", std::bind(&CanDriverNode::onSetZero, this, std::placeholders::_1, std::placeholders::_2));
        setZeroLimitSrv_ = create_service<can_driver::srv::SetZeroLimit>("set_zero_limit", std::bind(&CanDriverNode::onSetZeroLimit, this, std::placeholders::_1, std::placeholders::_2));
        applyLimitsSrv_ = create_service<can_driver::srv::ApplyLimits>("apply_limits", std::bind(&CanDriverNode::onApplyLimits, this, std::placeholders::_1, std::placeholders::_2));
        enableSrv_ = create_service<std_srvs::srv::Trigger>("enable", std::bind(&CanDriverNode::onEnable, this, std::placeholders::_1, std::placeholders::_2));
        disableSrv_ = create_service<std_srvs::srv::Trigger>("disable", std::bind(&CanDriverNode::onDisable, this, std::placeholders::_1, std::placeholders::_2));
        haltSrv_ = create_service<std_srvs::srv::Trigger>("halt", std::bind(&CanDriverNode::onHalt, this, std::placeholders::_1, std::placeholders::_2));
        resumeSrv_ = create_service<std_srvs::srv::Trigger>("resume", std::bind(&CanDriverNode::onResume, this, std::placeholders::_1, std::placeholders::_2));
    }

    void acceptDirect(std::size_t index, double value)
    {
        if (index >= axes_.size() || !std::isfinite(value)) return;
        std::lock_guard<std::mutex> lock(mutex_);
        axes_[index].directValue = value;
        axes_[index].hasDirect = true;
        axes_[index].lastCommand = std::chrono::steady_clock::now();
    }

    Axis *findAxis(std::uint16_t motorId)
    {
        for (auto &axis : axes_) if (can_driver::toSystemMotorId(axis.config.motorId) == motorId) return &axis;
        return nullptr;
    }

    std::shared_ptr<CanProtocol> protocolFor(const Axis &axis) const
    {
        return deviceManager_->getProtocol(axis.config.canDevice, axis.config.protocol);
    }

    bool initializeDevices(const std::string &filter, bool loopback)
    {
        std::map<std::string, std::vector<std::pair<CanType, MotorID>>> grouped;
        for (const auto &axis : axes_) {
            if (filter.empty() || axis.config.canDevice == filter) grouped[axis.config.canDevice].emplace_back(axis.config.protocol, axis.config.motorId);
        }
        deviceManager_->setRefreshRateHz(motorQueryHz_);
        deviceManager_->setPpFastWriteEnabled(ppFastWrite_);
        for (const auto &entry : grouped) if (!deviceManager_->initDevice(entry.first, entry.second, loopback)) return false;
        for (const auto &axis : axes_) {
            if (!filter.empty() && axis.config.canDevice != filter) continue;
            if (axis.config.protocol == CanType::DM && axis.config.dmMasterId != 0) {
                deviceManager_->configureDmMotorMasterId(axis.config.canDevice,
                                                         axis.config.motorId,
                                                         axis.config.dmMasterId);
            }
            if (auto protocol = protocolFor(axis)) {
                const auto mode = can_driver::axisControlModeFromString(axis.config.controlMode);
                const auto target = mode == can_driver::AxisControlMode::Velocity
                                         ? CanProtocol::MotorMode::Velocity
                                         : mode == can_driver::AxisControlMode::Csp
                                             ? CanProtocol::MotorMode::CSP
                                             : CanProtocol::MotorMode::Position;
                if (!protocol->setMode(axis.config.motorId, target)) return false;
            }
        }
        initialized_ = true;
        return true;
    }

    void shutdownDevices()
    {
        if (!deviceManager_) return;
        for (const auto &axis : axes_) if (auto protocol = protocolFor(axis)) {
            protocol->Stop(axis.config.motorId);
            protocol->Disable(axis.config.motorId);
        }
        deviceManager_->shutdownAll();
        initialized_ = false;
        state_ = "configured";
    }

    void publishState(const std::string &state)
    {
        state_ = state;
        if (lifecycleStatePub_) {
            std_msgs::msg::String message;
            message.data = state_;
            lifecycleStatePub_->publish(message);
        }
    }

    void controlLoop()
    {
        if (!initialized_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto &axis : axes_) {
            auto protocol = protocolFor(axis);
            if (!protocol) continue;
            axis.config.pos = can_driver::rawPositionToJointPosition(axis.config, protocol->getPosition(axis.config.motorId));
            axis.config.vel = static_cast<double>(protocol->getVelocity(axis.config.motorId)) * can_driver::effectiveVelocityScale(axis.config);
            if (state_ != "running" && !debugBypass_) continue;
            if (protocol->hasFault(axis.config.motorId)) {
                if (!axis.stopIssuedOnFault) { protocol->Stop(axis.config.motorId); axis.stopIssuedOnFault = true; }
                continue;
            }
            axis.stopIssuedOnFault = false;
            if (!axis.hasDirect || (!debugBypass_ && std::chrono::duration<double>(std::chrono::steady_clock::now() - axis.lastCommand).count() > directTimeoutSec_)) continue;
            const auto mode = can_driver::axisControlModeFromString(axis.config.controlMode);
            if (mode == can_driver::AxisControlMode::Velocity) {
                std::int32_t raw = 0;
                if (can_driver::safe_command::scaleAndClampToInt32(axis.directValue, can_driver::effectiveVelocityScale(axis.config), axis.config.name, raw)) protocol->setVelocity(axis.config.motorId, raw);
            } else {
                const auto raw = static_cast<std::int32_t>(std::llround(can_driver::jointPositionToRawCount(axis.config, axis.directValue)));
                if (mode == can_driver::AxisControlMode::Csp) protocol->quickSetPosition(axis.config.motorId, raw);
                else protocol->setPosition(axis.config.motorId, raw);
            }
        }
    }

    void publishMotorStates()
    {
        if (!initialized_ || !motorStatesPub_) return;
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto &axis : axes_) {
            auto protocol = protocolFor(axis);
            if (!protocol) continue;
            MotorState message;
            message.motor_id = can_driver::toSystemMotorId(axis.config.motorId);
            message.name = axis.config.name;
            const auto mode = can_driver::axisControlModeFromString(axis.config.controlMode);
            message.mode = mode == can_driver::AxisControlMode::Velocity
                               ? MotorState::MODE_VELOCITY
                               : mode == can_driver::AxisControlMode::Csp
                                   ? MotorState::MODE_CSP
                                   : MotorState::MODE_POSITION;
            message.enabled = protocol->isEnabled(axis.config.motorId);
            message.fault = protocol->hasFault(axis.config.motorId);
            message.position = static_cast<std::int32_t>(protocol->getPosition(axis.config.motorId));
            message.velocity = protocol->getVelocity(axis.config.motorId);
            message.current = protocol->getCurrent(axis.config.motorId);
            message.joint_position = axis.config.pos;
            message.joint_velocity = axis.config.vel;
            message.mode_valid = true;
            message.status_valid = true;
            message.position_valid = true;
            message.velocity_valid = true;
            message.current_valid = true;
            message.feedback_fresh = true;
            motorStatesPub_->publish(message);
        }
    }

    void onInit(const std::shared_ptr<can_driver::srv::Init::Request> request, std::shared_ptr<can_driver::srv::Init::Response> response)
    {
        const bool ok = initializeDevices(request->device, request->loopback);
        response->success = ok; response->message = ok ? "initialized" : "device initialization failed";
        if (ok) publishState("armed");
    }

    void onShutdown(const std::shared_ptr<can_driver::srv::Shutdown::Request>, std::shared_ptr<can_driver::srv::Shutdown::Response> response)
    {
        shutdownDevices(); response->success = true; response->message = "shutdown";
    }

    void onEnable(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        bool ok = initialized_;
        for (const auto &axis : axes_) { auto protocol = protocolFor(axis); ok = ok && protocol && protocol->Enable(axis.config.motorId); }
        response->success = ok; response->message = ok ? "enabled" : "enable failed"; if (ok) publishState("armed");
    }

    void onDisable(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        bool ok = true;
        for (const auto &axis : axes_) if (auto protocol = protocolFor(axis)) { protocol->Stop(axis.config.motorId); ok = protocol->Disable(axis.config.motorId) && ok; }
        response->success = ok; response->message = ok ? "disabled" : "disable failed"; if (ok) publishState("standby");
    }

    void onHalt(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        bool ok = true;
        for (const auto &axis : axes_) if (auto protocol = protocolFor(axis)) ok = protocol->Stop(axis.config.motorId) && ok;
        response->success = ok; response->message = ok ? "halted" : "halt failed"; if (ok) publishState("armed");
    }

    void onResume(const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        const bool ok = initialized_;
        if (ok) { std::lock_guard<std::mutex> lock(mutex_); for (auto &axis : axes_) axis.hasDirect = false; publishState("running"); }
        response->success = ok; response->message = ok ? "running" : "resume failed";
    }

    void onRecover(const std::shared_ptr<can_driver::srv::Recover::Request> request, std::shared_ptr<can_driver::srv::Recover::Response> response)
    {
        bool ok = false;
        for (const auto &axis : axes_) {
            if (request->motor_id != 0xffff && request->motor_id != can_driver::toSystemMotorId(axis.config.motorId)) continue;
            if (auto protocol = protocolFor(axis)) ok = protocol->ResetFault(axis.config.motorId) || ok;
        }
        response->success = ok; response->message = ok ? "recovered" : "no motor recovered"; if (ok) publishState("standby");
    }

    void onMotorCommand(const std::shared_ptr<can_driver::srv::MotorCommand::Request> request, std::shared_ptr<can_driver::srv::MotorCommand::Response> response)
    {
        auto *axis = findAxis(request->motor_id);
        if (!axis) { response->success = false; response->message = "unknown motor_id"; return; }
        auto protocol = protocolFor(*axis);
        if (!protocol) { response->success = false; response->message = "protocol unavailable"; return; }
        bool ok = false;
        switch (request->command) {
        case can_driver::srv::MotorCommand::Request::CMD_ENABLE: ok = protocol->Enable(axis->config.motorId); break;
        case can_driver::srv::MotorCommand::Request::CMD_DISABLE: ok = protocol->Disable(axis->config.motorId); break;
        case can_driver::srv::MotorCommand::Request::CMD_STOP: ok = protocol->Stop(axis->config.motorId); break;
        case can_driver::srv::MotorCommand::Request::CMD_SET_MODE: {
            const auto mode = request->value == 2.0 ? CanProtocol::MotorMode::CSP : request->value == 1.0 ? CanProtocol::MotorMode::Velocity : CanProtocol::MotorMode::Position;
            ok = protocol->setMode(axis->config.motorId, mode);
            if (ok) axis->config.controlMode = request->value == 2.0 ? "csp" : request->value == 1.0 ? "velocity" : "position";
            break;
        }
        default: response->success = false; response->message = "unknown command"; return;
        }
        response->success = ok; response->message = ok ? "ok" : "command rejected";
    }

    void onSetZero(const std::shared_ptr<can_driver::srv::SetZero::Request> request, std::shared_ptr<can_driver::srv::SetZero::Response> response)
    {
        auto *axis = findAxis(request->motor_id);
        if (!axis) { response->success = false; response->message = "unknown motor_id"; return; }
        response->previous_zero_offset_rad = axis->config.zeroOffsetRad; response->current_position_rad = axis->config.pos;
        axis->config.zeroOffsetRad = request->use_current_position_as_zero ? -axis->config.pos : request->zero_offset_rad;
        response->applied_zero_offset_rad = axis->config.zeroOffsetRad; response->success = true; response->message = "zero updated";
    }

    void onApplyLimits(const std::shared_ptr<can_driver::srv::ApplyLimits::Request> request, std::shared_ptr<can_driver::srv::ApplyLimits::Response> response)
    {
        auto *axis = findAxis(request->motor_id);
        if (!axis) { response->success = false; response->message = "unknown motor_id"; return; }
        axis->config.hasLimits = true; axis->config.limits.has_position_limits = true;
        axis->config.limits.min_position = request->min_position_rad; axis->config.limits.max_position = request->max_position_rad;
        response->success = true; response->message = "limits updated"; response->active_zero_offset_rad = axis->config.zeroOffsetRad;
        response->applied_min_rad = request->min_position_rad; response->applied_max_rad = request->max_position_rad; response->current_position_rad = axis->config.pos;
    }

    void onSetZeroLimit(const std::shared_ptr<can_driver::srv::SetZeroLimit::Request> request, std::shared_ptr<can_driver::srv::SetZeroLimit::Response> response)
    {
        auto *axis = findAxis(request->motor_id);
        if (!axis) { response->success = false; response->message = "unknown motor_id"; return; }
        axis->config.zeroOffsetRad = request->use_current_position_as_zero ? -axis->config.pos : request->zero_offset_rad;
        axis->config.hasLimits = true; axis->config.limits.has_position_limits = true;
        axis->config.limits.min_position = request->min_position_rad; axis->config.limits.max_position = request->max_position_rad;
        response->success = true; response->message = "zero and limits updated"; response->applied_zero_offset_rad = axis->config.zeroOffsetRad;
        response->applied_min_rad = request->min_position_rad; response->applied_max_rad = request->max_position_rad; response->current_position_rad = axis->config.pos;
    }

    std::string configFile_;
    double controlFrequency_{100.0};
    double statePeriodSec_{0.1};
    double directTimeoutSec_{0.5};
    double motorQueryHz_{0.0};
    bool debugBypass_{false};
    bool ppFastWrite_{false};
    bool initialized_{false};
    std::string state_{"configured"};
    std::mutex mutex_;
    std::shared_ptr<DeviceManager> deviceManager_;
    std::vector<Axis> axes_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> velSubs_;
    std::map<std::string, rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> posSubs_;
    rclcpp::Publisher<MotorState>::SharedPtr motorStatesPub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lifecycleStatePub_;
    rclcpp::TimerBase::SharedPtr controlTimer_;
    rclcpp::TimerBase::SharedPtr stateTimer_;
    rclcpp::Service<can_driver::srv::Init>::SharedPtr initSrv_;
    rclcpp::Service<can_driver::srv::Shutdown>::SharedPtr shutdownSrv_;
    rclcpp::Service<can_driver::srv::Recover>::SharedPtr recoverSrv_;
    rclcpp::Service<can_driver::srv::MotorCommand>::SharedPtr motorCmdSrv_;
    rclcpp::Service<can_driver::srv::SetZero>::SharedPtr setZeroSrv_;
    rclcpp::Service<can_driver::srv::SetZeroLimit>::SharedPtr setZeroLimitSrv_;
    rclcpp::Service<can_driver::srv::ApplyLimits>::SharedPtr applyLimitsSrv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enableSrv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disableSrv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr haltSrv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resumeSrv_;
};

} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
        auto node = std::make_shared<CanDriverNode>();
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &error) {
        std::fprintf(stderr, "[can_driver_node] %s\n", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
