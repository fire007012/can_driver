#include "can_driver/operational_coordinator.hpp"

#include <sstream>

#include <ros/ros.h>

namespace can_driver {

const char *SystemOpModeName(SystemOpMode mode)
{
    switch (mode) {
    case SystemOpMode::Inactive:
        return "Inactive";
    case SystemOpMode::Configured:
        return "Configured";
    case SystemOpMode::Standby:
        return "Standby";
    case SystemOpMode::Armed:
        return "Armed";
    case SystemOpMode::Running:
        return "Running";
    case SystemOpMode::Faulted:
        return "Faulted";
    case SystemOpMode::Recovering:
        return "Recovering";
    case SystemOpMode::ShuttingDown:
        return "ShuttingDown";
    }
    return "Unknown";
}

void OperationalCoordinator::ForceMode(SystemOpMode to, const char *reason)
{
    const SystemOpMode from = mode_.exchange(to, std::memory_order_acq_rel);
    if (from == to) {
        return;
    }

    if (reason && reason[0] != '\0') {
        ROS_INFO("[can_driver::OperationalCoordinator] %s -> %s (%s)",
                 SystemOpModeName(from), SystemOpModeName(to), reason);
    } else {
        ROS_INFO("[can_driver::OperationalCoordinator] %s -> %s",
                 SystemOpModeName(from), SystemOpModeName(to));
    }
}

void OperationalCoordinator::SetInactive()
{
    ForceMode(SystemOpMode::Inactive, "force");
}

void OperationalCoordinator::SetConfigured()
{
    ForceMode(SystemOpMode::Configured, "configured");
}

void OperationalCoordinator::SetFaulted()
{
    ForceMode(SystemOpMode::Faulted, "faulted");
}

OperationalCoordinator::Result OperationalCoordinator::DoTransition(
    std::initializer_list<SystemOpMode> allowedFrom, SystemOpMode to)
{
    std::lock_guard<std::mutex> lock(transitionMutex_);

    const SystemOpMode current = mode_.load(std::memory_order_acquire);
    bool allowed = false;
    for (const auto from : allowedFrom) {
        if (from == current) {
            allowed = true;
            break;
        }
    }

    if (!allowed) {
        if (current == to) {
            return {true, std::string("already ") + SystemOpModeName(to)};
        }

        std::ostringstream oss;
        oss << "cannot transition from " << SystemOpModeName(current)
            << " to " << SystemOpModeName(to);
        return {false, oss.str()};
    }

    mode_.store(to, std::memory_order_release);
    ROS_INFO("[can_driver::OperationalCoordinator] %s -> %s",
             SystemOpModeName(current), SystemOpModeName(to));
    return {true, std::string("-> ") + SystemOpModeName(to)};
}

OperationalCoordinator::Result OperationalCoordinator::RequestInit()
{
    return DoTransition({SystemOpMode::Configured}, SystemOpMode::Armed);
}

OperationalCoordinator::Result OperationalCoordinator::RequestEnable()
{
    return DoTransition({SystemOpMode::Standby}, SystemOpMode::Armed);
}

OperationalCoordinator::Result OperationalCoordinator::RequestDisable()
{
    return DoTransition({SystemOpMode::Standby, SystemOpMode::Armed, SystemOpMode::Running},
                        SystemOpMode::Standby);
}

OperationalCoordinator::Result OperationalCoordinator::RequestRelease()
{
    return DoTransition({SystemOpMode::Armed}, SystemOpMode::Running);
}

OperationalCoordinator::Result OperationalCoordinator::RequestHalt()
{
    return DoTransition({SystemOpMode::Running}, SystemOpMode::Armed);
}

OperationalCoordinator::Result OperationalCoordinator::RequestRecover()
{
    return DoTransition({SystemOpMode::Faulted}, SystemOpMode::Standby);
}

OperationalCoordinator::Result OperationalCoordinator::RequestShutdown()
{
    return DoTransition(
        {SystemOpMode::Inactive, SystemOpMode::Configured, SystemOpMode::Standby,
         SystemOpMode::Armed, SystemOpMode::Running, SystemOpMode::Faulted,
         SystemOpMode::Recovering, SystemOpMode::ShuttingDown},
        SystemOpMode::Configured);
}

} // namespace can_driver
