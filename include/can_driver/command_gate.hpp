#ifndef CAN_DRIVER_COMMAND_GATE_HPP
#define CAN_DRIVER_COMMAND_GATE_HPP

#include "can_driver/AxisCommandSemantics.h"

#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class CommandGate {
public:
    struct Snapshot {
        can_driver::AxisControlMode controlMode{can_driver::AxisControlMode::Unknown};
        double commandValue{0.0};
        bool hasDirectCommand{false};
        bool targetNearActual{false};
    };

    enum class DeviceEvent {
        None,
        Lost,
        Recovered
    };

    CommandGate() = default;

    void configure(std::function<std::vector<Snapshot>()> snapshotProvider,
                   std::function<void()> holdCallback);
    void reset();

    void holdCommands();
    void armFreshCommandLatch();
    bool consumeFreshCommandLatchIfSatisfied();
    DeviceEvent observeDeviceReady(const std::string &device, bool isReady);

private:
    std::function<std::vector<Snapshot>()> snapshotProvider_;
    std::function<void()> holdCallback_;

    std::vector<Snapshot> baselines_;
    std::map<std::string, bool> lastDeviceReadyState_;
    bool freshCommandRequired_{false};
    mutable std::mutex mutex_;
};

#endif // CAN_DRIVER_COMMAND_GATE_HPP
