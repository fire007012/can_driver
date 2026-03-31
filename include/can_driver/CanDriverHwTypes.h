#ifndef CAN_DRIVER_CAN_DRIVER_HW_TYPES_H
#define CAN_DRIVER_CAN_DRIVER_HW_TYPES_H

#include "can_driver/CanType.h"
#include "can_driver/MotorID.h"

#include <joint_limits_interface/joint_limits.h>
#include <ros/time.h>

#include <cstddef>
#include <string>
#include <vector>

namespace can_driver {

struct CanDriverJointConfig {
    std::string name;
    MotorID motorId{MotorID::LeftWheel};
    CanType protocol{CanType::MT};
    std::string canDevice;
    std::string controlMode;

    double positionScale{1.0};
    double velocityScale{1.0};

    double pos{0.0};
    double vel{0.0};
    double eff{0.0};
    double posCmd{0.0};
    double velCmd{0.0};

    double directPosCmd{0.0};
    double directVelCmd{0.0};
    bool hasDirectPosCmd{false};
    bool hasDirectVelCmd{false};
    ros::Time lastDirectPosTime;
    ros::Time lastDirectVelTime;

    joint_limits_interface::JointLimits limits;
    bool hasLimits{false};

    bool stopIssuedOnFault{false};
};

struct CanDriverDeviceProtocolGroup {
    std::string canDevice;
    CanType protocol{CanType::MT};
    std::vector<std::size_t> jointIndices;
};

} // namespace can_driver

#endif // CAN_DRIVER_CAN_DRIVER_HW_TYPES_H
