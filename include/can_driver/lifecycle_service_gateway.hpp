#ifndef CAN_DRIVER_LIFECYCLE_SERVICE_GATEWAY_HPP
#define CAN_DRIVER_LIFECYCLE_SERVICE_GATEWAY_HPP

#include <can_driver/Init.h>
#include <can_driver/Recover.h>
#include <can_driver/Shutdown.h>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>

namespace can_driver {
class OperationalCoordinator;
}

class LifecycleServiceGateway {
public:
    LifecycleServiceGateway() = default;
    LifecycleServiceGateway(ros::NodeHandle &pnh, can_driver::OperationalCoordinator *coordinator);

    void initialize(ros::NodeHandle &pnh, can_driver::OperationalCoordinator *coordinator);

private:
    bool onInit(can_driver::Init::Request &req, can_driver::Init::Response &res);
    bool onShutdown(can_driver::Shutdown::Request &req, can_driver::Shutdown::Response &res);
    bool onRecover(can_driver::Recover::Request &req, can_driver::Recover::Response &res);
    bool onEnable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onDisable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onHalt(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onResume(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    can_driver::OperationalCoordinator *coordinator_{nullptr};

    ros::ServiceServer initSrv_;
    ros::ServiceServer shutdownSrv_;
    ros::ServiceServer recoverSrv_;
    ros::ServiceServer enableSrv_;
    ros::ServiceServer disableSrv_;
    ros::ServiceServer haltSrv_;
    ros::ServiceServer resumeSrv_;
};

#endif // CAN_DRIVER_LIFECYCLE_SERVICE_GATEWAY_HPP
