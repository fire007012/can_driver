#include "can_driver/CanDriverRuntime.h"

#include <utility>

#include <ros/ros.h>

namespace can_driver {

CanDriverRuntime::CanDriverRuntime()
    : deviceManager_(std::make_shared<DeviceManager>())
{
    configureDependencies();
}

CanDriverRuntime::CanDriverRuntime(std::shared_ptr<IDeviceManager> deviceManager)
    : deviceManager_(std::move(deviceManager))
{
    if (!deviceManager_) {
        deviceManager_ = std::make_shared<DeviceManager>();
    }
    configureDependencies();
}

void CanDriverRuntime::configureDependencies()
{
    motorActionExecutor_.setDeviceManager(deviceManager_);
    lifecycleDriverOps_.configure(deviceManager_, &motorActionExecutor_);
}

void CanDriverRuntime::reset()
{
    active_.store(false, std::memory_order_release);
    lifecycleCoordinator_.SetInactive();
    deviceLoopbackByName_.clear();
    commandGate_.reset();
    lifecycleDriverOps_.setTargets({});
    if (deviceManager_) {
        deviceManager_->shutdownAll();
    }
}

void CanDriverRuntime::configureCommandGate(
    std::function<std::vector<CommandGate::Snapshot>()> snapshotProvider,
    std::function<void()> holdCallback)
{
    commandGate_.configure(std::move(snapshotProvider), std::move(holdCallback));
}

void CanDriverRuntime::configureLifecycleCoordinator(LifecycleHooks hooks)
{
    lifecycleHooks_ = std::move(hooks);

    OperationalCoordinator::DriverOps ops;
    ops.init_device = [this](const std::string &device, bool loopback) {
        return initializeLifecycleDevice(device, loopback);
    };
    ops.enable_all = [this]() {
        return lifecycleDriverOps_.enableAll();
    };
    ops.enable_healthy = [this](std::string *detail) {
        return lifecycleHooks_.enable_healthy ? lifecycleHooks_.enable_healthy(detail)
                                              : lifecycleDriverOps_.enableHealthy(detail);
    };
    ops.disable_all = [this]() {
        return lifecycleDriverOps_.disableAll();
    };
    ops.halt_all = [this]() {
        return lifecycleDriverOps_.haltAll();
    };
    ops.recover_all = [this]() {
        return recoverLifecycleDevices();
    };
    ops.shutdown_all = [this](bool force) {
        return shutdownLifecycleDriver(force);
    };
    ops.motion_healthy = [this](std::string *detail) {
        return lifecycleHooks_.motion_healthy ? lifecycleHooks_.motion_healthy(detail)
                                              : lifecycleDriverOps_.motionHealthy(detail);
    };
    ops.any_fault_active = [this]() {
        return lifecycleDriverOps_.anyFaultActive();
    };
    ops.hold_commands = [this]() {
        commandGate_.holdCommands();
    };
    ops.arm_fresh_command_latch = [this]() {
        commandGate_.armFreshCommandLatch();
    };
    lifecycleCoordinator_.SetDriverOps(std::move(ops));
}

OperationalCoordinator::Result CanDriverRuntime::initializeLifecycleDevice(
    const std::string &device,
    bool loopback)
{
    deviceLoopbackByName_[device] = loopback;

    const auto prepareResult = prepareLifecycleDeviceForStandby(device, loopback);
    if (!prepareResult.ok) {
        return prepareResult;
    }

    const auto enableResult = lifecycleDriverOps_.enableDevice(device);
    if (!enableResult.ok) {
        const auto rollback = lifecycleDriverOps_.shutdownDevice(device);
        if (!rollback.ok) {
            ROS_ERROR("[CanDriverRuntime] Failed to roll back prepared device '%s' after enable failure: %s",
                      device.c_str(),
                      rollback.message.c_str());
        }
        return enableResult;
    }
    active_.store(true, std::memory_order_release);
    return {true, "initialized (armed)"};
}

OperationalCoordinator::Result CanDriverRuntime::prepareLifecycleDeviceForStandby(
    const std::string &device,
    bool loopback)
{
    if (!lifecycleHooks_.sync_startup_position_and_commands ||
        !lifecycleHooks_.apply_pp_default_velocities ||
        !lifecycleHooks_.apply_initial_modes) {
        return {false, "prepare lifecycle path not available"};
    }

    const auto rollbackPreparedDevice =
        [this, &device](const OperationalCoordinator::Result &failure) {
            const auto rollback = lifecycleDriverOps_.shutdownDevice(device);
            if (!rollback.ok) {
                ROS_ERROR("[CanDriverRuntime] Failed to roll back prepared device '%s' after init failure: %s",
                          device.c_str(),
                          rollback.message.c_str());
            }
            return failure;
        };

    const double startupQueryHz =
        lifecycleHooks_.startup_query_hz ? lifecycleHooks_.startup_query_hz() : 0.0;
    const auto restoreSteadyRefresh = [this, &device]() {
        if (lifecycleHooks_.set_device_refresh_rate) {
            lifecycleHooks_.set_device_refresh_rate(device, 0.0);
        }
    };
    if (lifecycleHooks_.set_device_refresh_rate) {
        lifecycleHooks_.set_device_refresh_rate(device, startupQueryHz);
    }

    const auto prepareResult = lifecycleDriverOps_.prepareDevice(device, loopback);
    if (!prepareResult.ok) {
        restoreSteadyRefresh();
        return prepareResult;
    }

    if (lifecycleHooks_.apply_persisted_pp_zero_offsets &&
        !lifecycleHooks_.apply_persisted_pp_zero_offsets(device)) {
        restoreSteadyRefresh();
        return rollbackPreparedDevice(
            {false, "Failed to restore persisted PP zero offsets on " + device});
    }
    if (!lifecycleHooks_.sync_startup_position_and_commands(device)) {
        restoreSteadyRefresh();
        return rollbackPreparedDevice(
            {false, "Failed to synchronize startup position on " + device});
    }
    if (!lifecycleHooks_.apply_pp_default_velocities(device)) {
        restoreSteadyRefresh();
        return rollbackPreparedDevice(
            {false, "Failed to configure PP default velocities on " + device});
    }
    if (!lifecycleHooks_.apply_initial_modes(device)) {
        restoreSteadyRefresh();
        return rollbackPreparedDevice({false, "Failed to apply initial modes on " + device});
    }

    restoreSteadyRefresh();
    return {true, "prepared (standby)"};
}

OperationalCoordinator::Result CanDriverRuntime::recoverLifecycleDevices()
{
    if (!lifecycleHooks_.recover_devices) {
        return {false, "recover lifecycle path not available"};
    }

    const auto devices = lifecycleHooks_.recover_devices();
    if (devices.empty()) {
        return {false, "No devices available for recover."};
    }

    for (const auto &device : devices) {
        const auto loopbackIt = deviceLoopbackByName_.find(device);
        const bool loopback =
            (loopbackIt != deviceLoopbackByName_.end()) ? loopbackIt->second : false;
        const auto prepareResult = prepareLifecycleDeviceForStandby(device, loopback);
        if (!prepareResult.ok) {
            return prepareResult;
        }
    }

    const auto recoverResult = lifecycleDriverOps_.recoverAll();
    if (!recoverResult.ok) {
        return recoverResult;
    }

    active_.store(true, std::memory_order_release);
    return {true, "recovered (standby)"};
}

OperationalCoordinator::Result CanDriverRuntime::shutdownLifecycleDriver(bool force)
{
    active_.store(false, std::memory_order_release);
    const auto result = lifecycleDriverOps_.shutdownAll(force);

    if (lifecycleHooks_.clear_command_state) {
        lifecycleHooks_.clear_command_state();
    }

    if (result.ok) {
        ROS_INFO("[CanDriverRuntime] All devices shut down.");
    }
    return result;
}

} // namespace can_driver
