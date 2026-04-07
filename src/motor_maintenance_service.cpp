#include "can_driver/motor_maintenance_service.hpp"

#include "can_driver/AxisCommandSemantics.h"
#include "can_driver/EyouCan.h"
#include "can_driver/SafeCommand.h"

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <urdf/model.h>

#include <cmath>
#include <thread>

namespace {

struct ModeSelection {
    CanProtocol::MotorMode mode{CanProtocol::MotorMode::Position};
    can_driver::AxisControlMode controlMode{can_driver::AxisControlMode::Position};
};

bool decodeModeSelection(double value, ModeSelection *selection)
{
    if (selection == nullptr) {
        return false;
    }
    if (value == 0.0) {
        selection->controlMode = can_driver::AxisControlMode::Position;
        selection->mode = can_driver::protocolMotorModeFromAxisControlMode(selection->controlMode);
        return true;
    }
    if (value == 1.0) {
        selection->controlMode = can_driver::AxisControlMode::Velocity;
        selection->mode = can_driver::protocolMotorModeFromAxisControlMode(selection->controlMode);
        return true;
    }
    if (value == 2.0) {
        selection->controlMode = can_driver::AxisControlMode::Csp;
        selection->mode = can_driver::protocolMotorModeFromAxisControlMode(selection->controlMode);
        return true;
    }
    return false;
}

} // namespace

void MotorMaintenanceService::configure(
    ActivityChecker isActive,
    std::deque<JointConfig> *joints,
    std::map<std::string, std::size_t> *jointIndexByName,
    std::vector<uint8_t> *commandValidBuffer,
    std::map<uint16_t, double> *jointZeroOffsetRadByMotorId,
    std::mutex *jointStateMutex,
    MotorActionExecutor *motorActionExecutor,
    FreshFeedbackGetter getFreshAxisFeedback,
    DisabledRequirementChecker requireAxisDisabledForConfiguration,
    ProtocolGetter getProtocol,
    DeviceMutexGetter getDeviceMutex)
{
    isActive_ = std::move(isActive);
    joints_ = joints;
    jointIndexByName_ = jointIndexByName;
    commandValidBuffer_ = commandValidBuffer;
    jointZeroOffsetRadByMotorId_ = jointZeroOffsetRadByMotorId;
    jointStateMutex_ = jointStateMutex;
    motorActionExecutor_ = motorActionExecutor;
    getFreshAxisFeedback_ = std::move(getFreshAxisFeedback);
    requireAxisDisabledForConfiguration_ = std::move(requireAxisDisabledForConfiguration);
    getProtocol_ = std::move(getProtocol);
    getDeviceMutex_ = std::move(getDeviceMutex);
}

void MotorMaintenanceService::initialize(ros::NodeHandle &pnh)
{
    shutdown();
    motorCmdSrv_ = pnh.advertiseService("motor_command", &MotorMaintenanceService::onMotorCommand, this);
    setZeroLimitSrv_ =
        pnh.advertiseService("set_zero_limit", &MotorMaintenanceService::onSetZeroLimit, this);
}

void MotorMaintenanceService::shutdown()
{
    motorCmdSrv_.shutdown();
    setZeroLimitSrv_.shutdown();
}

const MotorMaintenanceService::JointConfig *
MotorMaintenanceService::findJointByMotorId(uint16_t motorId) const
{
    if (joints_ == nullptr) {
        return nullptr;
    }
    for (const auto &joint : *joints_) {
        if (static_cast<uint16_t>(joint.motorId) == motorId) {
            return &joint;
        }
    }
    return nullptr;
}

std::size_t MotorMaintenanceService::findJointIndexByMotorId(uint16_t motorId) const
{
    if (joints_ == nullptr) {
        return 0;
    }
    for (std::size_t index = 0; index < joints_->size(); ++index) {
        if (static_cast<uint16_t>((*joints_)[index].motorId) == motorId) {
            return index;
        }
    }
    return joints_->size();
}

MotorActionExecutor::Target
MotorMaintenanceService::makeMotorTarget(const JointConfig &joint) const
{
    return MotorActionExecutor::Target{joint.name, joint.canDevice, joint.protocol, joint.motorId};
}

void MotorMaintenanceService::clearDirectCmd(const std::string &jointName)
{
    if (joints_ == nullptr || jointIndexByName_ == nullptr || jointStateMutex_ == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(*jointStateMutex_);
    const auto it = jointIndexByName_->find(jointName);
    if (it != jointIndexByName_->end()) {
        (*joints_)[it->second].hasDirectPosCmd = false;
        (*joints_)[it->second].hasDirectVelCmd = false;
    }
}

bool MotorMaintenanceService::waitForPpModeConfirmation(const JointConfig &joint,
                                                        CanProtocol::MotorMode expectedMode,
                                                        std::string *message) const
{
    if (joint.protocol != CanType::PP || !getProtocol_) {
        return true;
    }

    auto protocol = std::dynamic_pointer_cast<EyouCan>(getProtocol_(joint.canDevice, joint.protocol));
    if (!protocol) {
        return true;
    }
    if (!getFreshAxisFeedback_) {
        if (message != nullptr) {
            *message = "Mode confirmation requires fresh feedback access.";
        }
        return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (std::chrono::steady_clock::now() < deadline) {
        protocol->issueRefreshQuery(joint.motorId, EyouCan::RefreshQuery::Mode);

        can_driver::SharedDriverState::AxisFeedbackState feedback;
        if (getFreshAxisFeedback_(joint, &feedback) && feedback.modeValid &&
            feedback.mode == expectedMode) {
            return true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (message != nullptr) {
        *message = "Timed out waiting for mode confirmation from PP feedback.";
    }
    return false;
}

bool MotorMaintenanceService::onMotorCommand(can_driver::MotorCommand::Request &req,
                                             can_driver::MotorCommand::Response &res)
{
    if (!isActive_ || !isActive_()) {
        res.success = false;
        res.message = "Driver inactive.";
        return true;
    }

    const auto *target = findJointByMotorId(req.motor_id);
    if (!target) {
        res.success = false;
        res.message = "Motor ID not found.";
        return true;
    }

    auto handleFailure = [&res](MotorActionExecutor::Status status, const char *rejectedMsg) {
        if (status == MotorActionExecutor::Status::DeviceNotReady) {
            res.success = false;
            res.message = "CAN device not ready.";
        } else if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
            res.success = false;
            res.message = "Protocol not available.";
        } else if (status == MotorActionExecutor::Status::Rejected) {
            res.success = false;
            res.message = rejectedMsg;
        } else {
            res.success = false;
            res.message = "Command execution failed.";
        }
    };

    if (req.command == can_driver::MotorCommand::Request::CMD_SET_MODE) {
        if (!motorActionExecutor_ || joints_ == nullptr || jointStateMutex_ == nullptr ||
            commandValidBuffer_ == nullptr) {
            res.success = false;
            res.message = "Motor maintenance service not configured.";
            return true;
        }

        ModeSelection selection;
        if (!decodeModeSelection(req.value, &selection)) {
            res.success = false;
            res.message = "CMD_SET_MODE value must be 0 (position), 1 (velocity) or 2 (csp).";
            return true;
        }
        if (!requireAxisDisabledForConfiguration_ ||
            !requireAxisDisabledForConfiguration_(*target, "Mode switch", &res.message)) {
            res.success = false;
            if (res.message.empty()) {
                res.message = "Mode switch requires the motor to be disabled first.";
            }
            return true;
        }

        const auto status = motorActionExecutor_->execute(
            makeMotorTarget(*target),
            [&selection](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                return proto->setMode(id, selection.mode);
            },
            "Set mode");
        if (status != MotorActionExecutor::Status::Ok) {
            handleFailure(status, "Set mode command rejected.");
            return true;
        }
        if (!waitForPpModeConfirmation(*target, selection.mode, &res.message)) {
            res.success = false;
            return true;
        }

        JointConfig targetSnapshot;
        const std::size_t targetIndex = findJointIndexByMotorId(req.motor_id);
        {
            std::lock_guard<std::mutex> lock(*jointStateMutex_);
            if (targetIndex >= joints_->size()) {
                res.success = false;
                res.message = "Motor ID not found.";
                return true;
            }
            targetSnapshot = (*joints_)[targetIndex];
        }

        int32_t preloadRaw = 0;
        if (selection.mode != CanProtocol::MotorMode::Velocity) {
            if (!getFreshAxisFeedback_) {
                res.success = false;
                res.message = "Current position feedback unavailable or stale for mode preload.";
                return true;
            }
            can_driver::SharedDriverState::AxisFeedbackState feedback;
            if (!getFreshAxisFeedback_(targetSnapshot, &feedback) || !feedback.positionValid) {
                res.success = false;
                res.message = "Current position feedback unavailable or stale for mode preload.";
                return true;
            }
            preloadRaw = can_driver::safe_command::clampToInt32(feedback.position);
        }

        const auto preloadStatus = motorActionExecutor_->execute(
            makeMotorTarget(targetSnapshot),
            [selection, preloadRaw](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                switch (selection.mode) {
                case CanProtocol::MotorMode::Position:
                    return proto->setPosition(id, preloadRaw);
                case CanProtocol::MotorMode::Velocity:
                    return proto->setVelocity(id, 0);
                case CanProtocol::MotorMode::CSP:
                    return proto->quickSetPosition(id, preloadRaw);
                }
                return false;
            },
            "Preload mode command");
        if (preloadStatus != MotorActionExecutor::Status::Ok) {
            handleFailure(preloadStatus, "Set mode preload command rejected.");
            return true;
        }

        {
            std::lock_guard<std::mutex> lock(*jointStateMutex_);
            auto &joint = (*joints_)[targetIndex];
            joint.controlMode = can_driver::axisControlModeName(selection.controlMode);
            joint.hasDirectPosCmd = false;
            joint.hasDirectVelCmd = false;
            joint.posCmd = joint.pos;
            joint.velCmd = 0.0;
            joint.requireCommandAlignment = true;
            (*commandValidBuffer_)[targetIndex] = 0;
        }
        res.success = true;
        res.message = "OK";
        return true;
    }

    struct CmdEntry {
        uint8_t cmd;
        const char *name;
        bool clearDirect;
        std::function<bool(const std::shared_ptr<CanProtocol> &, MotorID)> action;
    };
    const std::vector<CmdEntry> table = {
        {can_driver::MotorCommand::Request::CMD_ENABLE,
         "Enable",
         false,
         [](const std::shared_ptr<CanProtocol> &proto, MotorID id) { return proto->Enable(id); }},
        {can_driver::MotorCommand::Request::CMD_DISABLE,
         "Disable",
         true,
         [](const std::shared_ptr<CanProtocol> &proto, MotorID id) { return proto->Disable(id); }},
        {can_driver::MotorCommand::Request::CMD_STOP,
         "Stop",
         true,
         [](const std::shared_ptr<CanProtocol> &proto, MotorID id) { return proto->Stop(id); }},
    };

    for (const auto &entry : table) {
        if (req.command != entry.cmd) {
            continue;
        }
        if (!motorActionExecutor_) {
            res.success = false;
            res.message = "Motor maintenance service not configured.";
            return true;
        }
        const auto status =
            motorActionExecutor_->execute(makeMotorTarget(*target), entry.action, entry.name);
        if (status != MotorActionExecutor::Status::Ok) {
            const std::string rejectedMsg = std::string(entry.name) + " command rejected.";
            handleFailure(status, rejectedMsg.c_str());
            return true;
        }
        if (entry.clearDirect) {
            clearDirectCmd(target->name);
        }
        res.success = true;
        res.message = "OK";
        return true;
    }

    res.success = false;
    res.message = "Unknown command.";
    return true;
}

bool MotorMaintenanceService::onSetZeroLimit(can_driver::SetZeroLimit::Request &req,
                                             can_driver::SetZeroLimit::Response &res)
{
    if (!isActive_ || !isActive_()) {
        res.success = false;
        res.message = "Driver inactive.";
        return true;
    }

    const JointConfig *target = findJointByMotorId(req.motor_id);
    if (!target) {
        res.success = false;
        res.message = "Motor ID not found.";
        return true;
    }
    if (!can_driver::controlModeUsesPositionSemantics(target->controlMode)) {
        res.success = false;
        res.message = "Only position-semantic joints support zero/position limits.";
        return true;
    }
    if (req.use_current_position_as_zero && !req.apply_to_motor) {
        res.success = false;
        res.message = "Automatic zeroing requires apply_to_motor=true.";
        return true;
    }

    double baseMin = req.min_position_rad;
    double baseMax = req.max_position_rad;
    if (req.use_urdf_limits) {
        urdf::Model urdf;
        if (!urdf.initParam("robot_description")) {
            res.success = false;
            res.message = "robot_description not found; cannot read URDF limits.";
            return true;
        }
        auto urdfJoint = urdf.getJoint(target->name);
        if (!urdfJoint) {
            res.success = false;
            res.message = "Joint not found in URDF: " + target->name;
            return true;
        }
        joint_limits_interface::JointLimits urdfLimits;
        if (!joint_limits_interface::getJointLimits(urdfJoint, urdfLimits) ||
            !urdfLimits.has_position_limits) {
            res.success = false;
            res.message = "URDF has no position limits for joint: " + target->name;
            return true;
        }
        baseMin = urdfLimits.min_position;
        baseMax = urdfLimits.max_position;
    }

    if (!getProtocol_ || !getDeviceMutex_) {
        res.success = false;
        res.message = "Protocol not available.";
        return true;
    }
    auto proto = getProtocol_(target->canDevice, target->protocol);
    auto devMutex = getDeviceMutex_(target->canDevice);
    if (!proto || !devMutex) {
        res.success = false;
        res.message = "Protocol not available.";
        return true;
    }

    int64_t rawPos = 0;
    bool hasFreshFeedback = false;
    can_driver::SharedDriverState::AxisFeedbackState feedback;
    if (getFreshAxisFeedback_ && getFreshAxisFeedback_(*target, &feedback) &&
        feedback.positionValid) {
        rawPos = feedback.position;
        hasFreshFeedback = true;
    } else {
        std::lock_guard<std::mutex> devLock(*devMutex);
        for (int i = 0; i < 5; ++i) {
            rawPos = proto->getPosition(target->motorId);
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
    }

    const double currentPosRad = static_cast<double>(rawPos) * target->positionScale;
    res.current_position_rad = currentPosRad;
    if (req.use_current_position_as_zero && !hasFreshFeedback) {
        res.success = false;
        res.message = "Current position feedback unavailable or stale; cannot auto-zero.";
        return true;
    }

    const double resolvedZeroOffset =
        req.use_current_position_as_zero ? -currentPosRad : req.zero_offset_rad;
    res.applied_zero_offset_rad = resolvedZeroOffset;
    const double appliedMin = baseMin + resolvedZeroOffset;
    const double appliedMax = baseMax + resolvedZeroOffset;
    if (!std::isfinite(appliedMin) || !std::isfinite(appliedMax) || appliedMin >= appliedMax) {
        res.success = false;
        res.message = "Invalid limit range after zero offset.";
        return true;
    }
    if (currentPosRad < appliedMin || currentPosRad > appliedMax) {
        res.success = false;
        res.applied_min_rad = appliedMin;
        res.applied_max_rad = appliedMax;
        res.message = "Current position is outside requested limit range.";
        return true;
    }

    if (req.apply_to_motor) {
        if (!requireAxisDisabledForConfiguration_ ||
            !requireAxisDisabledForConfiguration_(
                *target, "Writing zero offset / hardware limits", &res.message)) {
            res.success = false;
            if (res.message.empty()) {
                res.message =
                    "Writing zero offset / hardware limits requires the motor to be disabled first.";
            }
            return true;
        }

        int32_t rawMin = 0;
        int32_t rawMax = 0;
        int32_t rawOffset = 0;
        if (!can_driver::safe_command::scaleAndClampToInt32(
                appliedMin, target->positionScale, target->name + ".min_limit", rawMin) ||
            !can_driver::safe_command::scaleAndClampToInt32(
                appliedMax, target->positionScale, target->name + ".max_limit", rawMax) ||
            !can_driver::safe_command::scaleAndClampToInt32(
                resolvedZeroOffset,
                target->positionScale,
                target->name + ".zero_offset",
                rawOffset)) {
            res.success = false;
            res.message = "Failed to convert limits/offset to protocol raw values.";
            return true;
        }

        if (!motorActionExecutor_) {
            res.success = false;
            res.message = "Motor maintenance service not configured.";
            return true;
        }
        const auto status = motorActionExecutor_->execute(
            makeMotorTarget(*target),
            [rawMin, rawMax, rawOffset](const std::shared_ptr<CanProtocol> &p, MotorID id) {
                const bool okOffset = p->setPositionOffset(id, rawOffset);
                const bool okLimits = p->configurePositionLimits(id, rawMin, rawMax, true);
                return okOffset && okLimits;
            },
            "Set zero/position limits");

        if (status != MotorActionExecutor::Status::Ok) {
            res.success = false;
            if (status == MotorActionExecutor::Status::DeviceNotReady) {
                res.message = "CAN device not ready.";
            } else if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
                res.message = "Protocol not available.";
            } else if (status == MotorActionExecutor::Status::Rejected) {
                res.message =
                    "Protocol rejected zero/limit settings (or unsupported by this protocol).";
            } else {
                res.message = "Set zero/limit execution failed.";
            }
            return true;
        }
    }

    if (joints_ && jointZeroOffsetRadByMotorId_ && jointStateMutex_) {
        std::lock_guard<std::mutex> lock(*jointStateMutex_);
        const auto targetIndex = findJointIndexByMotorId(req.motor_id);
        if (targetIndex < joints_->size()) {
            auto &joint = (*joints_)[targetIndex];
            joint.hasLimits = true;
            joint.limits.has_position_limits = true;
            joint.limits.min_position = appliedMin;
            joint.limits.max_position = appliedMax;
            (*jointZeroOffsetRadByMotorId_)[req.motor_id] = resolvedZeroOffset;
        }
    }

    res.success = true;
    res.applied_min_rad = appliedMin;
    res.applied_max_rad = appliedMax;
    res.message = "OK";
    return true;
}
