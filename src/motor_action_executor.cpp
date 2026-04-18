#include "can_driver/motor_action_executor.hpp"

#include <exception>

#include <ros/ros.h>

MotorActionExecutor::MotorActionExecutor(std::shared_ptr<IDeviceManager> deviceManager)
    : deviceManager_(std::move(deviceManager))
{
}

void MotorActionExecutor::setDeviceManager(std::shared_ptr<IDeviceManager> deviceManager)
{
    deviceManager_ = std::move(deviceManager);
}

MotorActionExecutor::Status MotorActionExecutor::execute(const Target &target,
                                                         const Action &action,
                                                         const char *operationName) const
{
    if (!deviceManager_ || !action) {
        return Status::ProtocolUnavailable;
    }
    if (!deviceManager_->isDeviceReady(target.canDevice)) {
        return Status::DeviceNotReady;
    }

    auto proto = deviceManager_->getProtocol(target.canDevice, target.protocol);
    auto devMutex = deviceManager_->getDeviceMutex(target.canDevice);
    if (!proto || !devMutex) {
        return Status::ProtocolUnavailable;
    }

    std::lock_guard<std::mutex> devLock(*devMutex);
    try {
        return action(proto, target.motorId) ? Status::Ok : Status::Rejected;
    } catch (const std::exception &e) {
        ROS_ERROR("[MotorActionExecutor] %s failed on '%s' motor %u: %s",
                  operationName,
                  target.canDevice.c_str(),
                  static_cast<unsigned>(static_cast<uint16_t>(target.motorId)),
                  e.what());
        return Status::Exception;
    } catch (...) {
        ROS_ERROR("[MotorActionExecutor] %s failed on '%s' motor %u (unknown exception).",
                  operationName,
                  target.canDevice.c_str(),
                  static_cast<unsigned>(static_cast<uint16_t>(target.motorId)));
        return Status::Exception;
    }
}

MotorActionExecutor::BatchResult MotorActionExecutor::executeBatch(const std::vector<Target> &targets,
                                                                   const Action &action,
                                                                   const char *operationName) const
{
    BatchResult result;
    for (const auto &target : targets) {
        const Status status = execute(target, action, operationName);
        if (status == Status::Ok) {
            result.anySuccess = true;
            ++result.successCount;
            result.succeededTargets.push_back(target);
            continue;
        }

        result.anyFailure = true;
        ++result.failureCount;
        if (result.firstFailure == Status::Ok) {
            result.firstFailure = status;
        }
        result.failures.push_back(BatchResult::Failure{target, status});
    }
    return result;
}
