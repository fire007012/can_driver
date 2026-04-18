#ifndef CAN_DRIVER_DRIVER_ROS_ENDPOINTS_HPP
#define CAN_DRIVER_DRIVER_ROS_ENDPOINTS_HPP

#include "can_driver/motor_maintenance_service.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <map>
#include <memory>
#include <string>

class CanDriverHW;

class DriverRosEndpoints {
public:
    DriverRosEndpoints(CanDriverHW &hw, ros::NodeHandle &pnh);

    void shutdown();
    void publishLifecycleStateNow();

private:
    void onDirectVelocity(std::size_t jointIndex, const std_msgs::Float64::ConstPtr &msg);
    void onDirectPosition(std::size_t jointIndex, const std_msgs::Float64::ConstPtr &msg);
    void onPublishMotorStates(const ros::TimerEvent &event);
    void onPublishLifecycleState(const ros::TimerEvent &event);

    CanDriverHW *hw_{nullptr};
    std::unique_ptr<MotorMaintenanceService> maintenanceService_;
    std::map<std::string, ros::Subscriber> cmdVelSubs_;
    std::map<std::string, ros::Subscriber> cmdPosSubs_;
    ros::Publisher motorStatesPub_;
    ros::Publisher lifecycleStatePub_;
    ros::Timer stateTimer_;
    ros::Timer lifecycleStateTimer_;
};

#endif // CAN_DRIVER_DRIVER_ROS_ENDPOINTS_HPP
