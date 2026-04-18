#ifndef CAN_DRIVER_MOTOR_ACTION_EXECUTOR_HPP
#define CAN_DRIVER_MOTOR_ACTION_EXECUTOR_HPP

#include "can_driver/CanType.h"
#include "can_driver/IDeviceManager.h"
#include "can_driver/MotorID.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

class MotorActionExecutor {
public:
    struct Target {
        std::string name;
        std::string canDevice;
        CanType protocol{CanType::MT};
        MotorID motorId{MotorID::LeftWheel};
    };

    enum class Status {
        Ok,
        DeviceNotReady,
        ProtocolUnavailable,
        Rejected,
        Exception
    };

    struct BatchResult {
        struct Failure {
            Target target;
            Status status{Status::Ok};
        };

        bool anySuccess{false};
        bool anyFailure{false};
        Status firstFailure{Status::Ok};
        std::size_t successCount{0};
        std::size_t failureCount{0};
        std::vector<Target> succeededTargets;
        std::vector<Failure> failures;
    };

    using Action = std::function<bool(const std::shared_ptr<CanProtocol> &, MotorID)>;

    MotorActionExecutor() = default;
    explicit MotorActionExecutor(std::shared_ptr<IDeviceManager> deviceManager);

    void setDeviceManager(std::shared_ptr<IDeviceManager> deviceManager);

    Status execute(const Target &target, const Action &action, const char *operationName) const;
    BatchResult executeBatch(const std::vector<Target> &targets,
                             const Action &action,
                             const char *operationName) const;

private:
    std::shared_ptr<IDeviceManager> deviceManager_;
};

#endif // CAN_DRIVER_MOTOR_ACTION_EXECUTOR_HPP
