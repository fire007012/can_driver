#ifndef CAN_DRIVER_SHARED_DRIVER_STATE_H
#define CAN_DRIVER_SHARED_DRIVER_STATE_H

#include "can_driver/CanProtocol.h"
#include "can_driver/CanType.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

namespace can_driver {

inline std::int64_t SharedDriverSteadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

inline std::uint16_t NormalizeProtocolMotorId(MotorID motorId)
{
    return static_cast<std::uint8_t>(motorId);
}

enum class AxisIntent : std::uint8_t {
    None = 0,
    Disable,
    Enable,
    Hold,
    Run,
    Recover,
};

class SharedDriverState {
public:
    struct AxisKey {
        std::string device;
        CanType protocol{CanType::MT};
        std::uint16_t motorId{0};
    };

    struct AxisFeedbackState {
        AxisKey key;
        std::int64_t position{0};
        std::int32_t velocity{0};
        std::int32_t current{0};
        CanProtocol::MotorMode mode{CanProtocol::MotorMode::Position};
        bool positionValid{false};
        bool velocityValid{false};
        bool currentValid{false};
        bool enabled{false};
        bool fault{false};
        bool feedbackSeen{false};
        bool degraded{false};
        std::int64_t lastRxSteadyNs{0};
        std::int64_t lastValidStateSteadyNs{0};
        std::int64_t lastModeMatchSteadyNs{0};
        std::int64_t lastEnableMatchSteadyNs{0};
        std::int64_t lastFaultSteadyNs{0};
        std::uint32_t consecutiveTimeoutCount{0};
    };

    struct AxisCommandState {
        AxisKey key;
        std::int64_t targetPosition{0};
        std::int32_t targetVelocity{0};
        std::int32_t targetCurrent{0};
        CanProtocol::MotorMode desiredMode{CanProtocol::MotorMode::Position};
        bool valid{false};
        std::uint64_t epoch{0};
        std::int64_t lastCommandSteadyNs{0};
    };

    struct DeviceHealthState {
        std::string device;
        bool transportReady{false};
        std::uint64_t txBackpressure{0};
        std::uint64_t txLinkUnavailable{0};
        std::uint64_t txError{0};
        std::uint64_t rxError{0};
        std::int64_t lastRxSteadyNs{0};
    };

    struct Snapshot {
        std::vector<AxisFeedbackState> axisFeedback;
        std::vector<AxisCommandState> axisCommands;
        std::vector<std::pair<AxisKey, AxisIntent>> axisIntents;
        std::vector<DeviceHealthState> deviceHealth;
        bool globalFault{false};
        bool degraded{false};
        std::uint64_t commandEpoch{0};
        std::uint64_t syncSequence{0};
    };

    void registerAxis(const AxisKey &key)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        touchAxisLocked(key);
    }

    void registerAxis(const std::string &device, CanType protocol, MotorID motorId)
    {
        registerAxis(MakeAxisKey(device, protocol, motorId));
    }

    bool hasAxis(const AxisKey &key) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return axisFeedback_.find(axisMapKey(key)) != axisFeedback_.end();
    }

    template <typename Fn>
    void mutateAxisFeedback(const AxisKey &key, Fn &&fn)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto mapKey = touchAxisLocked(key);
        auto &state = axisFeedback_[mapKey];
        fn(&state);
        recomputeAxisDerivedLocked(mapKey, SharedDriverSteadyNowNs());
        auto &deviceHealth = deviceHealth_[key.device];
        deviceHealth.device = key.device;
        if (state.lastRxSteadyNs > 0) {
            deviceHealth.lastRxSteadyNs =
                std::max(deviceHealth.lastRxSteadyNs, state.lastRxSteadyNs);
        }
    }

    bool getAxisFeedback(const AxisKey &key, AxisFeedbackState *out) const
    {
        if (!out) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = axisFeedback_.find(axisMapKey(key));
        if (it == axisFeedback_.end()) {
            return false;
        }
        *out = it->second;
        return true;
    }

    template <typename Fn>
    void mutateAxisCommand(const AxisKey &key, Fn &&fn)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto mapKey = touchAxisLocked(key);
        auto &state = axisCommands_[mapKey];
        fn(&state);
        recomputeAxisDerivedLocked(mapKey, SharedDriverSteadyNowNs());
    }

    bool getAxisCommand(const AxisKey &key, AxisCommandState *out) const
    {
        if (!out) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = axisCommands_.find(axisMapKey(key));
        if (it == axisCommands_.end()) {
            return false;
        }
        *out = it->second;
        return true;
    }

    void setAxisIntent(const AxisKey &key, AxisIntent intent)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto mapKey = touchAxisLocked(key);
        axisIntents_[mapKey] = intent;
        recomputeAxisDerivedLocked(mapKey, SharedDriverSteadyNowNs());
    }

    AxisIntent getAxisIntent(const AxisKey &key) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = axisIntents_.find(axisMapKey(key));
        return (it == axisIntents_.end()) ? AxisIntent::None : it->second;
    }

    template <typename Fn>
    void mutateDeviceHealth(const std::string &device, Fn &&fn)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto &health = deviceHealth_[device];
        health.device = device;
        fn(&health);
    }

    bool getDeviceHealth(const std::string &device, DeviceHealthState *out) const
    {
        if (!out) {
            return false;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = deviceHealth_.find(device);
        if (it == deviceHealth_.end()) {
            return false;
        }
        *out = it->second;
        return true;
    }

    void setGlobalFault(bool fault)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        globalFault_ = fault;
    }

    bool globalFault() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return globalFault_;
    }

    void advanceCommandEpoch()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++commandEpoch_;
    }

    std::uint64_t commandEpoch() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return commandEpoch_;
    }

    void advanceSyncSequence()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++syncSequence_;
    }

    std::uint64_t syncSequence() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return syncSequence_;
    }

    Snapshot snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Snapshot snapshot;
        snapshot.axisFeedback.reserve(axisFeedback_.size());
        snapshot.axisCommands.reserve(axisCommands_.size());
        snapshot.axisIntents.reserve(axisIntents_.size());
        snapshot.deviceHealth.reserve(deviceHealth_.size());

        for (const auto &entry : axisFeedback_) {
            snapshot.axisFeedback.push_back(entry.second);
        }
        for (const auto &entry : axisCommands_) {
            snapshot.axisCommands.push_back(entry.second);
        }
        for (const auto &entry : axisIntents_) {
            const auto feedbackIt = axisFeedback_.find(entry.first);
            if (feedbackIt != axisFeedback_.end()) {
                snapshot.axisIntents.emplace_back(feedbackIt->second.key, entry.second);
            }
        }
        for (const auto &entry : deviceHealth_) {
            snapshot.deviceHealth.push_back(entry.second);
        }

        snapshot.globalFault = globalFaultLocked();
        snapshot.degraded = degradedLocked();
        snapshot.commandEpoch = commandEpoch_;
        snapshot.syncSequence = syncSequence_;
        return snapshot;
    }

private:
    static AxisKey MakeAxisKey(const std::string &device, CanType protocol, MotorID motorId)
    {
        AxisKey key;
        key.device = device;
        key.protocol = protocol;
        key.motorId = NormalizeProtocolMotorId(motorId);
        return key;
    }

    static const char *protocolName(CanType protocol)
    {
        return protocol == CanType::MT ? "mt" : "pp";
    }

    static std::string axisMapKey(const AxisKey &key)
    {
        return key.device + "|" + protocolName(key.protocol) + "|" +
               std::to_string(key.motorId);
    }

    std::string touchAxisLocked(const AxisKey &key)
    {
        const auto mapKey = axisMapKey(key);
        auto &feedback = axisFeedback_[mapKey];
        feedback.key = key;
        auto &command = axisCommands_[mapKey];
        command.key = key;
        axisIntents_.emplace(mapKey, AxisIntent::None);
        auto &health = deviceHealth_[key.device];
        health.device = key.device;
        return mapKey;
    }

    bool intentWantsEnabledLocked(AxisIntent intent, bool *out) const
    {
        if (!out) {
            return false;
        }

        switch (intent) {
        case AxisIntent::Disable:
            *out = false;
            return true;
        case AxisIntent::Enable:
        case AxisIntent::Hold:
        case AxisIntent::Run:
        case AxisIntent::Recover:
            *out = true;
            return true;
        case AxisIntent::None:
        default:
            break;
        }
        return false;
    }

    void recomputeAxisDerivedLocked(const std::string &mapKey, std::int64_t nowNs)
    {
        auto feedbackIt = axisFeedback_.find(mapKey);
        if (feedbackIt == axisFeedback_.end()) {
            return;
        }

        AxisFeedbackState &feedback = feedbackIt->second;
        feedback.degraded = feedback.consecutiveTimeoutCount > 0;
        if (feedback.fault && nowNs > 0) {
            feedback.lastFaultSteadyNs = nowNs;
        }

        const auto commandIt = axisCommands_.find(mapKey);
        if (commandIt != axisCommands_.end()) {
            const auto &command = commandIt->second;
            if (command.valid && feedback.feedbackSeen && feedback.mode == command.desiredMode &&
                nowNs > 0) {
                feedback.lastModeMatchSteadyNs = nowNs;
            } else {
                feedback.lastModeMatchSteadyNs = 0;
            }
        } else {
            feedback.lastModeMatchSteadyNs = 0;
        }

        const auto intentIt = axisIntents_.find(mapKey);
        if (intentIt != axisIntents_.end()) {
            bool wantedEnabled = false;
            if (feedback.feedbackSeen &&
                intentWantsEnabledLocked(intentIt->second, &wantedEnabled) &&
                feedback.enabled == wantedEnabled && nowNs > 0) {
                feedback.lastEnableMatchSteadyNs = nowNs;
            }
        }
    }

    bool globalFaultLocked() const
    {
        if (globalFault_) {
            return true;
        }
        for (const auto &entry : axisFeedback_) {
            if (entry.second.fault) {
                return true;
            }
        }
        return false;
    }

    bool degradedLocked() const
    {
        for (const auto &entry : axisFeedback_) {
            if (entry.second.degraded) {
                return true;
            }
        }
        for (const auto &entry : deviceHealth_) {
            if (!entry.second.transportReady) {
                return true;
            }
        }
        return false;
    }

    mutable std::mutex mutex_;
    std::map<std::string, AxisFeedbackState> axisFeedback_;
    std::map<std::string, AxisCommandState> axisCommands_;
    std::map<std::string, AxisIntent> axisIntents_;
    std::map<std::string, DeviceHealthState> deviceHealth_;
    bool globalFault_{false};
    std::uint64_t commandEpoch_{0};
    std::uint64_t syncSequence_{0};
};

inline SharedDriverState::AxisKey MakeAxisKey(const std::string &device,
                                              CanType protocol,
                                              MotorID motorId)
{
    SharedDriverState::AxisKey key;
    key.device = device;
    key.protocol = protocol;
    key.motorId = NormalizeProtocolMotorId(motorId);
    return key;
}

} // namespace can_driver

#endif // CAN_DRIVER_SHARED_DRIVER_STATE_H
