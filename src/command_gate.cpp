#include "can_driver/command_gate.hpp"

#include <cmath>

namespace {

bool snapshotCommandChanged(const CommandGate::Snapshot &current,
                            const CommandGate::Snapshot &baseline)
{
    constexpr double kEps = 1e-12;

    if (current.controlMode != baseline.controlMode) {
        return true;
    }
    if (current.hasDirectCommand != baseline.hasDirectCommand) {
        return true;
    }
    return std::fabs(current.commandValue - baseline.commandValue) > kEps;
}

bool snapshotTargetAlreadyAligned(const CommandGate::Snapshot &snapshot)
{
    return snapshot.targetNearActual;
}

} // namespace

void CommandGate::configure(std::function<std::vector<Snapshot>()> snapshotProvider,
                            std::function<void()> holdCallback)
{
    std::lock_guard<std::mutex> lock(mutex_);
    snapshotProvider_ = std::move(snapshotProvider);
    holdCallback_ = std::move(holdCallback);
}

void CommandGate::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    baselines_.clear();
    lastDeviceReadyState_.clear();
    freshCommandRequired_ = false;
}

void CommandGate::holdCommands()
{
    std::function<void()> callback;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        callback = holdCallback_;
    }
    if (callback) {
        callback();
    }
}

void CommandGate::armFreshCommandLatch()
{
    std::function<std::vector<Snapshot>()> provider;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        provider = snapshotProvider_;
    }
    if (!provider) {
        return;
    }

    const auto snapshots = provider();
    std::lock_guard<std::mutex> lock(mutex_);
    baselines_ = snapshots;
    freshCommandRequired_ = true;
}

bool CommandGate::consumeFreshCommandLatchIfSatisfied()
{
    std::function<std::vector<Snapshot>()> provider;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!freshCommandRequired_) {
            return true;
        }
        provider = snapshotProvider_;
    }
    if (!provider) {
        std::lock_guard<std::mutex> lock(mutex_);
        freshCommandRequired_ = false;
        baselines_.clear();
        return true;
    }

    const auto snapshots = provider();

    std::lock_guard<std::mutex> lock(mutex_);
    if (baselines_.size() != snapshots.size()) {
        freshCommandRequired_ = false;
        baselines_.clear();
        return true;
    }

    bool satisfied = false;
    for (std::size_t i = 0; i < snapshots.size(); ++i) {
        const auto &current = snapshots[i];
        const auto &baseline = baselines_[i];
        if (snapshotCommandChanged(current, baseline) ||
            snapshotTargetAlreadyAligned(current)) {
            satisfied = true;
            break;
        }
    }

    if (!satisfied) {
        return false;
    }

    freshCommandRequired_ = false;
    return true;
}

CommandGate::DeviceEvent CommandGate::observeDeviceReady(const std::string &device, bool isReady)
{
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = lastDeviceReadyState_.find(device);
    const bool hadReadyState = (it != lastDeviceReadyState_.end());
    const bool wasReady = hadReadyState ? it->second : false;
    lastDeviceReadyState_[device] = isReady;

    if (!isReady) {
        return (!hadReadyState || wasReady) ? DeviceEvent::Lost : DeviceEvent::None;
    }
    if (hadReadyState && !wasReady) {
        return DeviceEvent::Recovered;
    }
    return DeviceEvent::None;
}
