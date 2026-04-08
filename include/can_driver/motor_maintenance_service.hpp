#ifndef CAN_DRIVER_MOTOR_MAINTENANCE_SERVICE_HPP
#define CAN_DRIVER_MOTOR_MAINTENANCE_SERVICE_HPP

#include "can_driver/CanDriverHwTypes.h"
#include "can_driver/CanProtocol.h"
#include "can_driver/AxisCommandSemantics.h"
#include "can_driver/motor_action_executor.hpp"
#include "can_driver/SharedDriverState.h"

#include <can_driver/MotorCommand.h>
#include <can_driver/SetZeroLimit.h>

#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <ros/ros.h>

class MotorMaintenanceService {
public:
    using JointConfig = can_driver::CanDriverJointConfig;
    using ActivityChecker = std::function<bool()>;
    using FreshFeedbackGetter = std::function<bool(
        const JointConfig &,
        can_driver::SharedDriverState::AxisFeedbackState *)>;
    using JointLookup = std::function<bool(uint16_t, JointConfig *)>;
    using DirectCommandClearer = std::function<void(const std::string &)>;
    using ModeSwitchCommitter =
        std::function<bool(uint16_t, can_driver::AxisControlMode)>;
    using LimitCommitter =
        std::function<bool(uint16_t, double, double, double)>;
    using DisabledRequirementChecker =
        std::function<bool(const JointConfig &, const char *, std::string *)>;
    using ProtocolGetter =
        std::function<std::shared_ptr<CanProtocol>(const std::string &, CanType)>;
    using DeviceMutexGetter =
        std::function<std::shared_ptr<std::mutex>(const std::string &)>;

    MotorMaintenanceService() = default;

    void configure(ActivityChecker isActive,
                   MotorActionExecutor *motorActionExecutor,
                   JointLookup lookupJointByMotorId,
                   DirectCommandClearer clearDirectCommand,
                   ModeSwitchCommitter commitModeSwitch,
                   LimitCommitter commitLimits,
                   FreshFeedbackGetter getFreshAxisFeedback,
                   DisabledRequirementChecker requireAxisDisabledForConfiguration,
                   ProtocolGetter getProtocol,
                   DeviceMutexGetter getDeviceMutex);

    void initialize(ros::NodeHandle &pnh);
    void shutdown();

private:
    bool lookupJointByMotorId(uint16_t motorId, JointConfig *joint) const;
    MotorActionExecutor::Target makeMotorTarget(const JointConfig &joint) const;
    bool waitForPpModeConfirmation(const JointConfig &joint,
                                   CanProtocol::MotorMode expectedMode,
                                   std::string *message) const;

    bool onMotorCommand(can_driver::MotorCommand::Request &req,
                        can_driver::MotorCommand::Response &res);
    bool onSetZeroLimit(can_driver::SetZeroLimit::Request &req,
                        can_driver::SetZeroLimit::Response &res);

    ActivityChecker isActive_;
    MotorActionExecutor *motorActionExecutor_{nullptr};
    JointLookup lookupJointByMotorId_;
    DirectCommandClearer clearDirectCommand_;
    ModeSwitchCommitter commitModeSwitch_;
    LimitCommitter commitLimits_;
    FreshFeedbackGetter getFreshAxisFeedback_;
    DisabledRequirementChecker requireAxisDisabledForConfiguration_;
    ProtocolGetter getProtocol_;
    DeviceMutexGetter getDeviceMutex_;

    ros::ServiceServer motorCmdSrv_;
    ros::ServiceServer setZeroLimitSrv_;
};

#endif // CAN_DRIVER_MOTOR_MAINTENANCE_SERVICE_HPP
