#ifndef CAN_DRIVER_DIRECT_COMMAND_TIMEOUT_H
#define CAN_DRIVER_DIRECT_COMMAND_TIMEOUT_H

#include "can_driver/CanProtocol.h"
#include <chrono>

namespace can_driver {

// Expiry must clear the protocol's cached command, because DM keepalive resends it.
// Keep an expired command pending if sending the stop failed, so the next cycle retries.
inline bool directCommandReady(CanProtocol &protocol, MotorID motorId, bool &pending,
                               std::chrono::steady_clock::time_point lastCommand,
                               double timeoutSec,
                               std::chrono::steady_clock::time_point now)
{
    if (!pending) return false;
    if (std::chrono::duration<double>(now - lastCommand).count() <= timeoutSec) return true;
    if (protocol.Stop(motorId)) pending = false;
    return false;
}

} // namespace can_driver
#endif
