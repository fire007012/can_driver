#ifndef CAN_DRIVER_MOTOR_MAINTENANCE_SERVICE_HPP
#define CAN_DRIVER_MOTOR_MAINTENANCE_SERVICE_HPP

#include "can_driver/CanDriverHwTypes.h"
#include "can_driver/CanProtocol.h"
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
    using DisabledRequirementChecker =
        std::function<bool(const JointConfig &, const char *, std::string *)>;
    using ProtocolGetter =
        std::function<std::shared_ptr<CanProtocol>(const std::string &, CanType)>;
    using DeviceMutexGetter =
        std::function<std::shared_ptr<std::mutex>(const std::string &)>;

    MotorMaintenanceService() = default;

    void configure(ActivityChecker isActive,
                   std::deque<JointConfig> *joints,
                   std::map<std::string, std::size_t> *jointIndexByName,
                   std::vector<uint8_t> *commandValidBuffer,
                   std::map<uint16_t, double> *jointZeroOffsetRadByMotorId,
                   std::mutex *jointStateMutex,
                   MotorActionExecutor *motorActionExecutor,
                   FreshFeedbackGetter getFreshAxisFeedback,
                   DisabledRequirementChecker requireAxisDisabledForConfiguration,
                   ProtocolGetter getProtocol,
                   DeviceMutexGetter getDeviceMutex);

    void initialize(ros::NodeHandle &pnh);
    void shutdown();

private:
    const JointConfig *findJointByMotorId(uint16_t motorId) const;
    std::size_t findJointIndexByMotorId(uint16_t motorId) const;
    MotorActionExecutor::Target makeMotorTarget(const JointConfig &joint) const;
    void clearDirectCmd(const std::string &jointName);

    bool onMotorCommand(can_driver::MotorCommand::Request &req,
                        can_driver::MotorCommand::Response &res);
    bool onSetZeroLimit(can_driver::SetZeroLimit::Request &req,
                        can_driver::SetZeroLimit::Response &res);

    ActivityChecker isActive_;
    std::deque<JointConfig> *joints_{nullptr};
    std::map<std::string, std::size_t> *jointIndexByName_{nullptr};
    std::vector<uint8_t> *commandValidBuffer_{nullptr};
    std::map<uint16_t, double> *jointZeroOffsetRadByMotorId_{nullptr};
    std::mutex *jointStateMutex_{nullptr};
    MotorActionExecutor *motorActionExecutor_{nullptr};
    FreshFeedbackGetter getFreshAxisFeedback_;
    DisabledRequirementChecker requireAxisDisabledForConfiguration_;
    ProtocolGetter getProtocol_;
    DeviceMutexGetter getDeviceMutex_;

    ros::ServiceServer motorCmdSrv_;
    ros::ServiceServer setZeroLimitSrv_;
};

#endif // CAN_DRIVER_MOTOR_MAINTENANCE_SERVICE_HPP
