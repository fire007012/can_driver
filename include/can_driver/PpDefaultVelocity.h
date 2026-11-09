#ifndef CAN_DRIVER_PP_DEFAULT_VELOCITY_H
#define CAN_DRIVER_PP_DEFAULT_VELOCITY_H

#include "can_driver/CanDriverHwTypes.h"
#include "can_driver/EyouCan.h"
#include <cmath>
#include <limits>
#include <stdexcept>

namespace can_driver {

// Magnitude in joint SI units; direction is deliberately not applied here.
inline std::int32_t ppDefaultVelocityRaw(double velocity, double scale)
{
    const double raw = velocity / scale;
    if (!std::isfinite(velocity) || velocity <= 0 || !std::isfinite(scale) || scale <= 0 ||
        !std::isfinite(raw) || raw < 0.5 || raw > std::numeric_limits<std::int32_t>::max())
        throw std::invalid_argument("invalid PP default velocity or velocity scale");
    return static_cast<std::int32_t>(std::llround(raw));
}

inline void validatePpDefaultVelocities(const CanDriverJointConfig &config)
{
    if (config.hasPpPositionDefaultVelocity)
        ppDefaultVelocityRaw(config.ppPositionDefaultVelocity, config.velocityScale);
    if (config.hasPpCspDefaultVelocity)
        ppDefaultVelocityRaw(config.ppCspDefaultVelocity, config.velocityScale);
}

inline void applyPpDefaultVelocities(EyouCan &protocol, const CanDriverJointConfig &config)
{
    if (config.hasPpPositionDefaultVelocity)
        protocol.setMotorDefaultPositionVelocityRaw(config.motorId,
            ppDefaultVelocityRaw(config.ppPositionDefaultVelocity, config.velocityScale));
    if (config.hasPpCspDefaultVelocity)
        protocol.setMotorDefaultCspVelocityRaw(config.motorId,
            ppDefaultVelocityRaw(config.ppCspDefaultVelocity, config.velocityScale));
}

} // namespace can_driver
#endif
