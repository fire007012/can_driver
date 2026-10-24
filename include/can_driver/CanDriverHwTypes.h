#ifndef CAN_DRIVER_CAN_DRIVER_HW_TYPES_H
#define CAN_DRIVER_CAN_DRIVER_HW_TYPES_H

#include "can_driver/CanType.h"
#include "can_driver/MotorID.h"
#include "can_driver/ros2_compat.hpp"

#include <chrono>
#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>

namespace can_driver {

struct JointLimits {
    bool has_position_limits{false};
    double min_position{0.0};
    double max_position{0.0};
    bool has_velocity_limits{false};
    double max_velocity{0.0};
    bool has_effort_limits{false};
    double max_effort{0.0};
};

enum class PreparedCommandRoute {
    Position,
    Velocity,
    Csp,
};

struct CanDriverPreparedCommand {
    bool valid{false};
    std::size_t jointIndex{0};
    MotorID motorId{MotorID::LeftWheel};
    PreparedCommandRoute route{PreparedCommandRoute::Position};
    std::string jointName;
};

struct CanDriverJointConfig {
    std::string name;
    MotorID motorId{MotorID::LeftWheel};
    CanType protocol{CanType::MT};
    std::string canDevice;
    std::string controlMode;
    std::uint32_t dmMasterId{0};

    double positionScale{1.0};
    double velocityScale{1.0};
    double directionSign{1.0};
    double ppPositionDefaultVelocity{0.0};
    double ppCspDefaultVelocity{0.0};
    bool hasPpPositionDefaultVelocity{false};
    bool hasPpCspDefaultVelocity{false};
    double ipMaxVelocity{1.0};
    double ipMaxAcceleration{2.0};
    double ipMaxJerk{10.0};
    double ipGoalTolerance{1e-3};
    std::string ecbIp;
    bool ecbAutoDiscovery{false};
    int ecbRefreshMs{20};

    double pos{0.0};
    double vel{0.0};
    double eff{0.0};
    double posCmd{0.0};
    double velCmd{0.0};
    double zeroOffsetRad{0.0};

    double directPosCmd{0.0};
    double directVelCmd{0.0};
    bool hasDirectPosCmd{false};
    bool hasDirectVelCmd{false};
    SteadyTimePoint lastDirectPosTime{};
    SteadyTimePoint lastDirectVelTime{};

    JointLimits limits;
    bool hasLimits{false};

    bool stopIssuedOnFault{false};
    bool requireCommandAlignment{false};
    bool startupPositionOutsideLimits{false};
};

struct CanDriverDeviceProtocolGroup {
    std::string canDevice;
    CanType protocol{CanType::MT};
    std::vector<std::size_t> jointIndices;
};

} // namespace can_driver

#endif // CAN_DRIVER_CAN_DRIVER_HW_TYPES_H
