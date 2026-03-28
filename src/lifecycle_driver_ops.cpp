#include "can_driver/lifecycle_driver_ops.hpp"

#include <chrono>
#include <thread>
#include <utility>

namespace can_driver {

LifecycleDriverOps::LifecycleDriverOps(std::shared_ptr<IDeviceManager> deviceManager,
                                       const MotorActionExecutor *motorActionExecutor)
    : deviceManager_(std::move(deviceManager)),
      motorActionExecutor_(motorActionExecutor)
{
}

void LifecycleDriverOps::configure(std::shared_ptr<IDeviceManager> deviceManager,
                                   const MotorActionExecutor *motorActionExecutor)
{
    std::lock_guard<std::mutex> lock(targetsMutex_);
    deviceManager_ = std::move(deviceManager);
    motorActionExecutor_ = motorActionExecutor;
}

void LifecycleDriverOps::setTargets(std::vector<MotorActionExecutor::Target> targets)
{
    std::lock_guard<std::mutex> lock(targetsMutex_);
    targets_ = std::move(targets);
}

std::vector<MotorActionExecutor::Target> LifecycleDriverOps::targetsSnapshot() const
{
    std::lock_guard<std::mutex> lock(targetsMutex_);
    return targets_;
}

LifecycleDriverOps::Result LifecycleDriverOps::makeMotorActionFailureResult(
    MotorActionExecutor::Status status,
    const char *rejectedMessage,
    const char *protocolUnavailableMessage) const
{
    if (status == MotorActionExecutor::Status::DeviceNotReady) {
        return {false, "CAN device not ready."};
    }
    if (status == MotorActionExecutor::Status::ProtocolUnavailable) {
        return {false, protocolUnavailableMessage ? protocolUnavailableMessage : "Protocol not available."};
    }
    if (status == MotorActionExecutor::Status::Rejected) {
        return {false, rejectedMessage ? rejectedMessage : "Command rejected."};
    }
    return {false, "Command execution failed."};
}

LifecycleDriverOps::Result LifecycleDriverOps::runMotorBatchAction(
    const std::vector<MotorActionExecutor::Target> &targets,
    const MotorActionExecutor::Action &action,
    const char *operationName,
    const char *rejectedMessage,
    const char *protocolUnavailableMessage,
    bool requireAnyTarget) const
{
    if (!motorActionExecutor_) {
        return {false, "Motor action executor unavailable."};
    }
    if (targets.empty()) {
        return requireAnyTarget
                   ? Result{false, "No joints available."}
                   : Result{true, "No joints to process."};
    }

    const auto batch = motorActionExecutor_->executeBatch(targets, action, operationName);
    if (!batch.anySuccess && batch.anyFailure) {
        return makeMotorActionFailureResult(batch.firstFailure,
                                            rejectedMessage,
                                            protocolUnavailableMessage);
    }
    if (!batch.anySuccess && requireAnyTarget) {
        return {false, "No joints available."};
    }
    if (batch.anyFailure) {
        return makeMotorActionFailureResult(batch.firstFailure,
                                            rejectedMessage,
                                            protocolUnavailableMessage);
    }
    return {true, "OK"};
}

std::shared_ptr<CanProtocol> LifecycleDriverOps::getProtocol(const std::string &device,
                                                             CanType type) const
{
    if (!deviceManager_) {
        return nullptr;
    }
    return deviceManager_->getProtocol(device, type);
}

std::shared_ptr<std::mutex> LifecycleDriverOps::getDeviceMutex(const std::string &device) const
{
    if (!deviceManager_) {
        return nullptr;
    }
    return deviceManager_->getDeviceMutex(device);
}

bool LifecycleDriverOps::isDeviceReady(const std::string &device) const
{
    return deviceManager_ && deviceManager_->isDeviceReady(device);
}

bool LifecycleDriverOps::queryMotorFault(const MotorActionExecutor::Target &target,
                                         bool *hasFault) const
{
    if (!hasFault) {
        return false;
    }

    auto proto = getProtocol(target.canDevice, target.protocol);
    auto devMutex = getDeviceMutex(target.canDevice);
    if (!proto || !devMutex) {
        return false;
    }

    std::lock_guard<std::mutex> devLock(*devMutex);
    *hasFault = proto->hasFault(target.motorId);
    return true;
}

LifecycleDriverOps::Result LifecycleDriverOps::initializeDevice(const std::string &device,
                                                                bool loopback) const
{
    if (!deviceManager_) {
        return {false, "Device manager unavailable."};
    }

    std::vector<std::pair<CanType, MotorID>> motors;
    for (const auto &target : targetsSnapshot()) {
        if (target.canDevice != device) {
            continue;
        }
        motors.emplace_back(target.protocol, target.motorId);
    }

    if (!deviceManager_->initDevice(device, motors, loopback)) {
        return {false, "Failed to initialize " + device};
    }
    return {true, "initialized (armed)"};
}

LifecycleDriverOps::Result LifecycleDriverOps::enableAll() const
{
    if (!motorActionExecutor_) {
        return {false, "Motor action executor unavailable."};
    }

    const auto targets = targetsSnapshot();
    if (targets.empty()) {
        return {false, "No joints available for enable."};
    }

    const auto batch = motorActionExecutor_->executeBatch(
        targets,
        [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->Enable(id);
        },
        "Enable");
    if (batch.anyFailure) {
        if (!batch.succeededTargets.empty()) {
            const auto rollback = motorActionExecutor_->executeBatch(
                batch.succeededTargets,
                [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
                    return proto->Disable(id);
                },
                "Enable rollback");
            if (rollback.anyFailure) {
                return makeMotorActionFailureResult(
                    rollback.firstFailure,
                    "Enable rollback failed after partial success.",
                    "Protocol not available during enable rollback.");
            }
        }
        return makeMotorActionFailureResult(batch.firstFailure,
                                            "Enable command rejected.",
                                            "Protocol not available.");
    }
    if (!batch.anySuccess) {
        return {false, "No joints available for enable."};
    }
    return {true, "enabled (armed)"};
}

LifecycleDriverOps::Result LifecycleDriverOps::disableAll() const
{
    return runMotorBatchAction(
        targetsSnapshot(),
        [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->Disable(id);
        },
        "Disable",
        "Disable command rejected.",
        "Protocol not available.",
        false);
}

LifecycleDriverOps::Result LifecycleDriverOps::haltAll() const
{
    return runMotorBatchAction(
        targetsSnapshot(),
        [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->Stop(id);
        },
        "Halt",
        "Halt command rejected.",
        "Protocol not available.",
        false);
}

LifecycleDriverOps::Result LifecycleDriverOps::recoverAll() const
{
    if (!motorActionExecutor_) {
        return {false, "Motor action executor unavailable."};
    }

    const auto targets = targetsSnapshot();
    if (targets.empty()) {
        return {false, "No motors available for recover."};
    }

    const auto batch = motorActionExecutor_->executeBatch(
        targets,
        [](const std::shared_ptr<CanProtocol> &proto, MotorID id) {
            return proto->ResetFault(id);
        },
        "Recover");
    if (!batch.anySuccess && batch.anyFailure) {
        return makeMotorActionFailureResult(batch.firstFailure,
                                            "Recover command rejected.",
                                            "Protocol fault-reset path not available.");
    }
    if (!batch.anySuccess) {
        return {false, "No motors available for recover."};
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        bool allHealthy = true;
        for (const auto &target : targets) {
            bool hasFault = false;
            if (!queryMotorFault(target, &hasFault)) {
                return {false, "Protocol not available during fault verification."};
            }
            if (hasFault) {
                allHealthy = false;
                break;
            }
        }
        if (allHealthy) {
            return {true, "Recovered (standby)."};
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    return {false, "Recover timeout: fault still active."};
}

LifecycleDriverOps::Result LifecycleDriverOps::shutdownAll(bool force) const
{
    (void)force;
    if (!deviceManager_) {
        return {false, "Device manager unavailable."};
    }

    deviceManager_->shutdownAll();
    return {true, "All CAN devices shut down."};
}

bool LifecycleDriverOps::anyFaultActive() const
{
    for (const auto &target : targetsSnapshot()) {
        bool hasFault = false;
        if (queryMotorFault(target, &hasFault) && hasFault) {
            return true;
        }
    }
    return false;
}

bool LifecycleDriverOps::motionHealthy(std::string *detail) const
{
    for (const auto &target : targetsSnapshot()) {
        if (!isDeviceReady(target.canDevice)) {
            if (detail) {
                *detail = "CAN device not ready.";
            }
            return false;
        }
        bool hasFault = false;
        if (!queryMotorFault(target, &hasFault)) {
            if (detail) {
                *detail = "Protocol not available.";
            }
            return false;
        }
        if (hasFault) {
            if (detail) {
                *detail = "Fault still active.";
            }
            return false;
        }
    }
    return true;
}

} // namespace can_driver
