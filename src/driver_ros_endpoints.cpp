#include "can_driver/driver_ros_endpoints.hpp"

#include "can_driver/CanDriverHW.h"

#include <std_msgs/String.h>

DriverRosEndpoints::DriverRosEndpoints(CanDriverHW &hw, ros::NodeHandle &pnh)
    : hw_(&hw)
{
    maintenanceService_ = std::make_unique<MotorMaintenanceService>();
    hw_->configureMotorMaintenanceService(*maintenanceService_);
    maintenanceService_->initialize(pnh);

    lifecycleStatePub_ = pnh.advertise<std_msgs::String>("lifecycle_state", 1, true);
    lifecycleStateTimer_ =
        pnh.createTimer(ros::Duration(0.1), &DriverRosEndpoints::onPublishLifecycleState, this);

    for (const auto &endpoint : hw_->directCommandEndpoints()) {
        const std::string velTopic = "motor/" + endpoint.jointName + "/cmd_velocity";
        const std::string posTopic = "motor/" + endpoint.jointName + "/cmd_position";
        cmdVelSubs_[endpoint.jointName] = pnh.subscribe<std_msgs::Float64>(
            velTopic, static_cast<uint32_t>(hw_->directCommandQueueSize()),
            [this, endpoint](const std_msgs::Float64::ConstPtr &msg) {
                onDirectVelocity(endpoint.jointIndex, msg);
            });
        cmdPosSubs_[endpoint.jointName] = pnh.subscribe<std_msgs::Float64>(
            posTopic, static_cast<uint32_t>(hw_->directCommandQueueSize()),
            [this, endpoint](const std_msgs::Float64::ConstPtr &msg) {
                onDirectPosition(endpoint.jointIndex, msg);
            });
    }

    motorStatesPub_ = pnh.advertise<can_driver::MotorState>("motor_states", 10);
    stateTimer_ = pnh.createTimer(ros::Duration(hw_->motorStatePeriodSec()),
                                  &DriverRosEndpoints::onPublishMotorStates,
                                  this);
}

void DriverRosEndpoints::shutdown()
{
    stateTimer_.stop();
    lifecycleStateTimer_.stop();
    for (auto &kv : cmdVelSubs_) {
        kv.second.shutdown();
    }
    for (auto &kv : cmdPosSubs_) {
        kv.second.shutdown();
    }
    cmdVelSubs_.clear();
    cmdPosSubs_.clear();
    if (maintenanceService_) {
        maintenanceService_->shutdown();
    }
    motorStatesPub_.shutdown();
    lifecycleStatePub_.shutdown();
}

void DriverRosEndpoints::publishLifecycleStateNow()
{
    if (hw_ != nullptr) {
        hw_->publishLifecycleState(lifecycleStatePub_);
    }
}

void DriverRosEndpoints::onDirectVelocity(std::size_t jointIndex,
                                          const std_msgs::Float64::ConstPtr &msg)
{
    if (hw_ == nullptr || msg == nullptr) {
        return;
    }
    hw_->acceptDirectCommand(jointIndex, true, msg->data, ros::Time::now());
}

void DriverRosEndpoints::onDirectPosition(std::size_t jointIndex,
                                          const std_msgs::Float64::ConstPtr &msg)
{
    if (hw_ == nullptr || msg == nullptr) {
        return;
    }
    hw_->acceptDirectCommand(jointIndex, false, msg->data, ros::Time::now());
}

void DriverRosEndpoints::onPublishMotorStates(const ros::TimerEvent & /*event*/)
{
    if (hw_ != nullptr) {
        hw_->publishMotorStates(motorStatesPub_);
    }
}

void DriverRosEndpoints::onPublishLifecycleState(const ros::TimerEvent & /*event*/)
{
    if (hw_ != nullptr) {
        hw_->publishLifecycleState(lifecycleStatePub_);
    }
}
