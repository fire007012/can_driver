#ifndef CAN_DRIVER_CAN_DRIVER_IO_RUNTIME_H
#define CAN_DRIVER_CAN_DRIVER_IO_RUNTIME_H

#include "can_driver/AxisReadinessEvaluator.h"
#include "can_driver/CanDriverHwTypes.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/SafeCommand.h"
#include "can_driver/SharedDriverState.h"
#include "can_driver/command_gate.hpp"

#include <can_driver/MotorState.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <exception>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>

namespace can_driver {

class CanDriverIoRuntime {
public:
    struct WriteConfig {
        double directCmdTimeoutSec{0.5};
        bool debugBypassRosControl{false};
        bool safetyStopOnFault{true};
        bool safetyRequireEnabledForMotion{true};
        double maxPositionStepRad{0.0};
        bool safetyHoldAfterDeviceRecover{true};
    };

    struct PublishResult {
        std::vector<can_driver::MotorState> messages;
        bool anyFault{false};
    };

    static void SyncJointFeedback(const IDeviceManager &deviceManager,
                                  const std::vector<CanDriverDeviceProtocolGroup> &groups,
                                  std::deque<CanDriverJointConfig> *joints,
                                  std::mutex *jointStateMutex)
    {
        if (joints == nullptr || jointStateMutex == nullptr) {
            return;
        }

        struct JointSnapshot {
            double pos{0.0};
            double vel{0.0};
            double eff{0.0};
            bool valid{false};
        };

        std::vector<JointSnapshot> snapshots(joints->size());
        for (const auto &group : groups) {
            auto proto = deviceManager.getProtocol(group.canDevice, group.protocol);
            auto devMutex = deviceManager.getDeviceMutex(group.canDevice);
            if (!proto || !devMutex) {
                continue;
            }

            std::lock_guard<std::mutex> devLock(*devMutex);
            for (const std::size_t index : group.jointIndices) {
                const auto &joint = (*joints)[index];
                snapshots[index].pos =
                    static_cast<double>(proto->getPosition(joint.motorId)) * joint.positionScale;
                snapshots[index].vel =
                    static_cast<double>(proto->getVelocity(joint.motorId)) * joint.velocityScale;
                snapshots[index].eff = static_cast<double>(proto->getCurrent(joint.motorId));
                snapshots[index].valid = true;
            }
        }

        std::lock_guard<std::mutex> lock(*jointStateMutex);
        for (std::size_t index = 0; index < joints->size(); ++index) {
            if (!snapshots[index].valid) {
                continue;
            }
            (*joints)[index].pos = snapshots[index].pos;
            (*joints)[index].vel = snapshots[index].vel;
            (*joints)[index].eff = snapshots[index].eff;
        }
    }

    static void PrepareCommands(std::deque<CanDriverJointConfig> *joints,
                                std::vector<int32_t> *rawCommandBuffer,
                                std::vector<uint8_t> *commandValidBuffer,
                                std::mutex *jointStateMutex,
                                const WriteConfig &config)
    {
        if (joints == nullptr || rawCommandBuffer == nullptr || commandValidBuffer == nullptr ||
            jointStateMutex == nullptr) {
            return;
        }

        std::lock_guard<std::mutex> lock(*jointStateMutex);
        const ros::Time now = ros::Time::now();
        for (std::size_t index = 0; index < joints->size(); ++index) {
            auto &joint = (*joints)[index];

            bool useDirect = false;
            double cmdValue = 0.0;
            bool hasCommand = true;
            if (joint.controlMode == "velocity") {
                if (joint.hasDirectVelCmd) {
                    if (config.debugBypassRosControl) {
                        useDirect = true;
                        cmdValue = joint.directVelCmd;
                    } else {
                        const double age = (now - joint.lastDirectVelTime).toSec();
                        if (age <= config.directCmdTimeoutSec) {
                            useDirect = true;
                            cmdValue = joint.directVelCmd;
                        } else {
                            joint.hasDirectVelCmd = false;
                        }
                    }
                }
                if (!useDirect) {
                    if (config.debugBypassRosControl) {
                        hasCommand = false;
                    } else {
                        cmdValue = joint.velCmd;
                    }
                }
            } else {
                if (joint.hasDirectPosCmd) {
                    if (config.debugBypassRosControl) {
                        useDirect = true;
                        cmdValue = joint.directPosCmd;
                    } else {
                        const double age = (now - joint.lastDirectPosTime).toSec();
                        if (age <= config.directCmdTimeoutSec) {
                            useDirect = true;
                            cmdValue = joint.directPosCmd;
                        } else {
                            joint.hasDirectPosCmd = false;
                        }
                    }
                }
                if (!useDirect) {
                    if (config.debugBypassRosControl) {
                        hasCommand = false;
                    } else {
                        cmdValue = joint.posCmd;
                    }
                }
            }

            if (!hasCommand) {
                (*commandValidBuffer)[index] = 0;
                continue;
            }

            cmdValue = clampWithJointLimits(joint, cmdValue);
            if (joint.controlMode == "position" && config.maxPositionStepRad > 0.0 &&
                std::isfinite(cmdValue) && std::isfinite(joint.pos)) {
                const double delta = cmdValue - joint.pos;
                if (std::fabs(delta) > config.maxPositionStepRad) {
                    const double limitedCmd =
                        joint.pos + std::copysign(config.maxPositionStepRad, delta);
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[CanDriverHW] Joint '%s' position step limited: cur=%.6f, req=%.6f, limited=%.6f (max_step=%.6f).",
                        joint.name.c_str(),
                        joint.pos,
                        cmdValue,
                        limitedCmd,
                        config.maxPositionStepRad);
                    cmdValue = limitedCmd;
                }
                cmdValue = clampWithJointLimits(joint, cmdValue);
            }

            const double scale =
                (joint.controlMode == "velocity") ? joint.velocityScale : joint.positionScale;
            (*commandValidBuffer)[index] = static_cast<uint8_t>(
                can_driver::safe_command::scaleAndClampToInt32(
                    cmdValue, scale, joint.name, (*rawCommandBuffer)[index]));
        }
    }

    static void DispatchPreparedCommands(const IDeviceManager &deviceManager,
                                         const std::vector<CanDriverDeviceProtocolGroup> &groups,
                                         std::deque<CanDriverJointConfig> *joints,
                                         const std::vector<int32_t> &rawCommandBuffer,
                                         std::vector<uint8_t> *commandValidBuffer,
                                         std::mutex *jointStateMutex,
                                         CommandGate *commandGate,
                                         const WriteConfig &config,
                                         bool *anyFaultObserved)
    {
        if (joints == nullptr || commandValidBuffer == nullptr || jointStateMutex == nullptr ||
            commandGate == nullptr) {
            return;
        }

        struct MotionStatusSnapshot {
            bool enabled{false};
            bool fault{false};
            bool valid{false};
        };

        const auto sharedState = deviceManager.getSharedDriverState();

        for (const auto &group : groups) {
            const bool isReady = deviceManager.isDeviceReady(group.canDevice);
            const auto deviceEvent = commandGate->observeDeviceReady(group.canDevice, isReady);

            if (!isReady) {
                if (deviceEvent == CommandGate::DeviceEvent::Lost) {
                    ROS_ERROR_THROTTLE(
                        1.0,
                        "[CanDriverHW] Device '%s' not ready, clear direct commands and block motion.",
                        group.canDevice.c_str());
                } else {
                    ROS_WARN_THROTTLE(
                        1.0,
                        "[CanDriverHW] Device '%s' not ready, skip command write.",
                        group.canDevice.c_str());
                }

                clearGroupCommands(group, joints, commandValidBuffer, jointStateMutex);
                continue;
            }

            if (config.safetyHoldAfterDeviceRecover &&
                deviceEvent == CommandGate::DeviceEvent::Recovered) {
                ROS_WARN_THROTTLE(
                    1.0,
                    "[CanDriverHW] Device '%s' recovered, hold one cycle and require fresh command.",
                    group.canDevice.c_str());
                commandGate->holdCommands();
                commandGate->armFreshCommandLatch();
                continue;
            }

            std::vector<MotionStatusSnapshot> motionStatusSnapshots(joints->size());
            if (sharedState) {
                const auto nowNs = SharedDriverSteadyNowNs();
                for (const std::size_t index : group.jointIndices) {
                    const auto &joint = (*joints)[index];
                    SharedDriverState::AxisFeedbackState feedback;
                    if (!sharedState->getAxisFeedback(
                            MakeAxisKey(group.canDevice, group.protocol, joint.motorId),
                            &feedback) ||
                        !sharedFeedbackFresh(feedback, nowNs)) {
                        continue;
                    }

                    motionStatusSnapshots[index].enabled = feedback.enabled;
                    motionStatusSnapshots[index].fault = feedback.fault;
                    motionStatusSnapshots[index].valid = true;
                }
            }

            auto proto = deviceManager.getProtocol(group.canDevice, group.protocol);
            auto devMutex = deviceManager.getDeviceMutex(group.canDevice);
            if (!proto || !devMutex) {
                continue;
            }

            std::lock_guard<std::mutex> devLock(*devMutex);
            for (const std::size_t index : group.jointIndices) {
                if (!(*commandValidBuffer)[index]) {
                    continue;
                }

                const auto &joint = (*joints)[index];
                try {
                    const bool hasSharedMotionStatus = motionStatusSnapshots[index].valid;
                    const bool hasFault = hasSharedMotionStatus
                                              ? motionStatusSnapshots[index].fault
                                              : proto->hasFault(joint.motorId);

                    if (config.safetyStopOnFault) {
                        bool needIssueStop = false;
                        {
                            std::lock_guard<std::mutex> lock(*jointStateMutex);
                            if (hasFault) {
                                if (!(*joints)[index].stopIssuedOnFault) {
                                    (*joints)[index].stopIssuedOnFault = true;
                                    needIssueStop = true;
                                }
                            } else {
                                (*joints)[index].stopIssuedOnFault = false;
                            }
                        }

                        if (hasFault) {
                            if (anyFaultObserved != nullptr) {
                                *anyFaultObserved = true;
                            }
                            if (needIssueStop) {
                                if (!proto->Stop(joint.motorId)) {
                                    ROS_WARN_THROTTLE(
                                        1.0,
                                        "[CanDriverHW] Joint '%s' has fault, auto Stop rejected on '%s'.",
                                        joint.name.c_str(),
                                        group.canDevice.c_str());
                                } else {
                                    ROS_WARN_THROTTLE(
                                        1.0,
                                        "[CanDriverHW] Joint '%s' has fault, auto Stop sent and motion command blocked.",
                                        joint.name.c_str());
                                }
                            }
                            continue;
                        }
                    }

                    const bool enabled = hasSharedMotionStatus
                                             ? motionStatusSnapshots[index].enabled
                                             : proto->isEnabled(joint.motorId);
                    if (config.safetyRequireEnabledForMotion && !enabled) {
                        ROS_WARN_THROTTLE(
                            1.0,
                            "[CanDriverHW] Joint '%s' is not enabled, motion command blocked.",
                            joint.name.c_str());
                        continue;
                    }

                    if (joint.controlMode == "velocity") {
                        if (!proto->setVelocity(joint.motorId, rawCommandBuffer[index])) {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "[CanDriverHW] setVelocity rejected on '%s'.",
                                group.canDevice.c_str());
                        }
                    } else if (joint.controlMode == "csp") {
                        if (!proto->quickSetPosition(joint.motorId, rawCommandBuffer[index])) {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "[CanDriverHW] quickSetPosition rejected on '%s'.",
                                group.canDevice.c_str());
                        }
                    } else {
                        if (!proto->setPosition(joint.motorId, rawCommandBuffer[index])) {
                            ROS_WARN_THROTTLE(
                                1.0,
                                "[CanDriverHW] setPosition rejected on '%s'.",
                                group.canDevice.c_str());
                        }
                    }
                } catch (const std::exception &e) {
                    ROS_ERROR_THROTTLE(1.0,
                                       "[CanDriverHW] write() command failed on '%s': %s",
                                       group.canDevice.c_str(),
                                       e.what());
                } catch (...) {
                    ROS_ERROR_THROTTLE(
                        1.0,
                        "[CanDriverHW] write() command failed on '%s' (unknown exception).",
                        group.canDevice.c_str());
                }
            }
        }
    }

    static PublishResult BuildMotorStateMessages(
        const IDeviceManager &deviceManager,
        const std::vector<CanDriverDeviceProtocolGroup> &groups,
        const std::deque<CanDriverJointConfig> &joints,
        std::mutex *jointStateMutex)
    {
        PublishResult result;
        result.messages.reserve(joints.size());

        struct StatusSnapshot {
            bool enabled{false};
            bool fault{false};
            std::uint8_t mode{can_driver::MotorState::MODE_UNKNOWN};
            bool statusValid{false};
            bool modeValid{false};
        };

        std::vector<StatusSnapshot> statusSnapshots(joints.size());
        const auto sharedState = deviceManager.getSharedDriverState();
        if (sharedState) {
            for (const auto &group : groups) {
                for (const std::size_t index : group.jointIndices) {
                    const auto &joint = joints[index];
                    can_driver::SharedDriverState::AxisFeedbackState feedback;
                    if (!sharedState->getAxisFeedback(
                            can_driver::MakeAxisKey(group.canDevice, group.protocol, joint.motorId),
                            &feedback) ||
                        !feedback.feedbackSeen) {
                        continue;
                    }

                    statusSnapshots[index].enabled = feedback.enabled;
                    statusSnapshots[index].fault = feedback.fault;
                    statusSnapshots[index].mode = motorStateModeFromProtocolMode(feedback.mode);
                    statusSnapshots[index].statusValid = true;
                    statusSnapshots[index].modeValid = true;
                }
            }
        }

        for (const auto &group : groups) {
            auto proto = deviceManager.getProtocol(group.canDevice, group.protocol);
            auto devMutex = deviceManager.getDeviceMutex(group.canDevice);
            if (!proto || !devMutex) {
                continue;
            }

            std::lock_guard<std::mutex> devLock(*devMutex);
            for (const std::size_t index : group.jointIndices) {
                if (statusSnapshots[index].statusValid) {
                    continue;
                }
                const auto &joint = joints[index];
                statusSnapshots[index].enabled = proto->isEnabled(joint.motorId);
                statusSnapshots[index].fault = proto->hasFault(joint.motorId);
                statusSnapshots[index].statusValid = true;
            }
        }

        std::lock_guard<std::mutex> lock(*jointStateMutex);
        for (std::size_t index = 0; index < joints.size(); ++index) {
            const auto &joint = joints[index];
            can_driver::MotorState message;
            message.motor_id = static_cast<uint16_t>(joint.motorId);
            message.name = joint.name;

            const double rawPos = joint.pos / joint.positionScale;
            const double rawVel = joint.vel / joint.velocityScale;
            message.position = can_driver::safe_command::clampToInt32(rawPos);
            message.velocity = can_driver::safe_command::clampToInt16(rawVel);
            message.current = can_driver::safe_command::clampToInt16(joint.eff);
            message.mode = statusSnapshots[index].modeValid
                               ? statusSnapshots[index].mode
                               : motorStateModeFromControlMode(joint.controlMode);
            if (statusSnapshots[index].statusValid) {
                message.enabled = statusSnapshots[index].enabled;
                message.fault = statusSnapshots[index].fault;
            }
            result.anyFault = result.anyFault || message.fault;

            result.messages.push_back(std::move(message));
        }

        return result;
    }

private:
    static std::uint8_t motorStateModeFromProtocolMode(CanProtocol::MotorMode mode)
    {
        switch (mode) {
        case CanProtocol::MotorMode::Velocity:
            return can_driver::MotorState::MODE_VELOCITY;
        case CanProtocol::MotorMode::CSP:
            return can_driver::MotorState::MODE_CSP;
        case CanProtocol::MotorMode::Position:
            return can_driver::MotorState::MODE_POSITION;
        }
        return can_driver::MotorState::MODE_UNKNOWN;
    }

    static std::uint8_t motorStateModeFromControlMode(const std::string &controlMode)
    {
        if (controlMode == "velocity") {
            return can_driver::MotorState::MODE_VELOCITY;
        }
        if (controlMode == "csp") {
            return can_driver::MotorState::MODE_CSP;
        }
        return can_driver::MotorState::MODE_POSITION;
    }

    static bool sharedFeedbackFresh(const SharedDriverState::AxisFeedbackState &feedback,
                                    std::int64_t nowNs)
    {
        if (!feedback.feedbackSeen || feedback.lastRxSteadyNs <= 0) {
            return false;
        }

        const AxisReadinessEvaluator::Config config;
        if (config.feedbackFreshnessTimeoutNs <= 0 || nowNs <= feedback.lastRxSteadyNs) {
            return true;
        }
        return (nowNs - feedback.lastRxSteadyNs) <= config.feedbackFreshnessTimeoutNs;
    }

    static double clampWithJointLimits(const CanDriverJointConfig &joint, double cmdValue)
    {
        if (!joint.hasLimits || !std::isfinite(cmdValue)) {
            return cmdValue;
        }
        if (joint.controlMode == "velocity" && joint.limits.has_velocity_limits) {
            return std::clamp(cmdValue,
                              -joint.limits.max_velocity,
                              joint.limits.max_velocity);
        }
        if (joint.controlMode == "position" && joint.limits.has_position_limits) {
            return std::clamp(cmdValue,
                              joint.limits.min_position,
                              joint.limits.max_position);
        }
        return cmdValue;
    }

    static void clearGroupCommands(const CanDriverDeviceProtocolGroup &group,
                                   std::deque<CanDriverJointConfig> *joints,
                                   std::vector<uint8_t> *commandValidBuffer,
                                   std::mutex *jointStateMutex)
    {
        std::lock_guard<std::mutex> lock(*jointStateMutex);
        for (const std::size_t index : group.jointIndices) {
            auto &joint = (*joints)[index];
            joint.hasDirectPosCmd = false;
            joint.hasDirectVelCmd = false;
            joint.stopIssuedOnFault = false;
            if (joint.controlMode == "velocity") {
                joint.velCmd = 0.0;
            } else {
                joint.posCmd = joint.pos;
            }
            (*commandValidBuffer)[index] = 0;
        }
    }
};

} // namespace can_driver

#endif // CAN_DRIVER_CAN_DRIVER_IO_RUNTIME_H
