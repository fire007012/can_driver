#ifndef CAN_DRIVER_OPERATIONAL_COORDINATOR_HPP
#define CAN_DRIVER_OPERATIONAL_COORDINATOR_HPP

#include <atomic>
#include <initializer_list>
#include <mutex>
#include <string>

namespace can_driver {

enum class SystemOpMode : unsigned char {
    Inactive,
    Configured,
    Standby,
    Armed,
    Running,
    Faulted,
    Recovering,
    ShuttingDown,
};

const char *SystemOpModeName(SystemOpMode mode);

class OperationalCoordinator {
public:
    struct Result {
        bool ok{false};
        std::string message;
    };

    OperationalCoordinator() = default;

    SystemOpMode mode() const
    {
        return mode_.load(std::memory_order_acquire);
    }

    void SetInactive();
    void SetConfigured();
    void SetFaulted();

    Result RequestInit();
    Result RequestEnable();
    Result RequestDisable();
    Result RequestRelease();
    Result RequestHalt();
    Result RequestRecover();
    Result RequestShutdown();

private:
    Result DoTransition(std::initializer_list<SystemOpMode> allowedFrom,
                        SystemOpMode to);
    void ForceMode(SystemOpMode to, const char *reason);

    std::atomic<SystemOpMode> mode_{SystemOpMode::Inactive};
    mutable std::mutex transitionMutex_;
};

} // namespace can_driver

#endif // CAN_DRIVER_OPERATIONAL_COORDINATOR_HPP
