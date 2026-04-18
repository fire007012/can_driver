#ifndef CAN_DRIVER_CAN_DRIVER_RUNTIME_H
#define CAN_DRIVER_CAN_DRIVER_RUNTIME_H

#include "can_driver/DeviceManager.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/command_gate.hpp"
#include "can_driver/lifecycle_driver_ops.hpp"
#include "can_driver/motor_action_executor.hpp"
#include "can_driver/operational_coordinator.hpp"

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace can_driver {

class CanDriverRuntime {
public:
    struct LifecycleHooks {
        std::function<std::vector<std::string>()> recover_devices;
        std::function<void()> clear_command_state;
        std::function<bool(std::string *)> enable_healthy;
        std::function<bool(std::string *)> motion_healthy;
        std::function<double()> startup_query_hz;
        std::function<void(const std::string &, double)> set_device_refresh_rate;
        std::function<bool(const std::string &)> apply_persisted_pp_zero_offsets;
        std::function<bool(const std::string &)> sync_startup_position_and_commands;
        std::function<bool(const std::string &)> apply_pp_default_velocities;
        std::function<bool(const std::string &)> apply_initial_modes;
    };

    CanDriverRuntime();
    explicit CanDriverRuntime(std::shared_ptr<IDeviceManager> deviceManager);

    std::shared_ptr<IDeviceManager> &deviceManager()
    {
        return deviceManager_;
    }

    MotorActionExecutor &motorActionExecutor()
    {
        return motorActionExecutor_;
    }

    LifecycleDriverOps &lifecycleDriverOps()
    {
        return lifecycleDriverOps_;
    }

    CommandGate &commandGate()
    {
        return commandGate_;
    }

    OperationalCoordinator &lifecycleCoordinator()
    {
        return lifecycleCoordinator_;
    }

    std::atomic<bool> &activeFlag()
    {
        return active_;
    }

    std::map<std::string, bool> &deviceLoopbackByName()
    {
        return deviceLoopbackByName_;
    }

    void reset();
    void configureCommandGate(std::function<std::vector<CommandGate::Snapshot>()> snapshotProvider,
                              std::function<void()> holdCallback);
    void configureLifecycleCoordinator(LifecycleHooks hooks);

private:
    void configureDependencies();
    OperationalCoordinator::Result prepareLifecycleDeviceForStandby(const std::string &device, bool loopback);
    OperationalCoordinator::Result initializeLifecycleDevice(const std::string &device, bool loopback);
    OperationalCoordinator::Result recoverLifecycleDevices();
    OperationalCoordinator::Result shutdownLifecycleDriver(bool force);

    std::shared_ptr<IDeviceManager> deviceManager_;
    MotorActionExecutor motorActionExecutor_;
    LifecycleDriverOps lifecycleDriverOps_;
    CommandGate commandGate_;
    std::atomic<bool> active_{false};
    OperationalCoordinator lifecycleCoordinator_;
    std::map<std::string, bool> deviceLoopbackByName_;
    LifecycleHooks lifecycleHooks_;
};

} // namespace can_driver

#endif // CAN_DRIVER_CAN_DRIVER_RUNTIME_H
