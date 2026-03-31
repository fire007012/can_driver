#ifndef CAN_DRIVER_AXIS_RUNTIME_H
#define CAN_DRIVER_AXIS_RUNTIME_H

#include "can_driver/SharedDriverState.h"

#include <cstdint>
#include <string>

namespace can_driver {

enum class AxisRuntimeState : std::uint8_t {
    Offline = 0,
    Seen,
    Standby,
    Armed,
    Running,
    Faulted,
    Recovering,
};

inline const char *AxisRuntimeStateName(AxisRuntimeState state)
{
    switch (state) {
    case AxisRuntimeState::Offline:
        return "Offline";
    case AxisRuntimeState::Seen:
        return "Seen";
    case AxisRuntimeState::Standby:
        return "Standby";
    case AxisRuntimeState::Armed:
        return "Armed";
    case AxisRuntimeState::Running:
        return "Running";
    case AxisRuntimeState::Faulted:
        return "Faulted";
    case AxisRuntimeState::Recovering:
        return "Recovering";
    }
    return "Unknown";
}

struct AxisRuntimeStatus {
    SharedDriverState::AxisKey key;
    AxisRuntimeState state{AxisRuntimeState::Offline};
    AxisIntent intent{AxisIntent::None};
    bool deviceReady{false};
    bool feedbackSeen{false};
    bool feedbackFresh{false};
    bool degraded{false};
    bool fault{false};
    bool faultCleared{false};
    bool enabled{false};
    bool enabledReady{false};
    bool commandValid{false};
    bool modeExpected{false};
    bool modeMatched{true};
    bool modeReady{true};
    bool feedbackReady{false};
    bool axisReadyForEnable{false};
    bool axisReadyForRun{false};
    bool recoverConfirmed{false};
};

class AxisRuntime {
public:
    struct Config {
        std::int64_t feedbackFreshnessTimeoutNs{500000000LL};
        std::uint32_t recoverConfirmCycles{2};
    };

    AxisRuntime() = default;
    explicit AxisRuntime(Config config)
        : config_(config)
    {
    }

    AxisRuntimeStatus Evaluate(
        const SharedDriverState::AxisFeedbackState &feedback,
        const SharedDriverState::AxisCommandState *command,
        AxisIntent intent,
        const SharedDriverState::DeviceHealthState *deviceHealth,
        std::int64_t nowNs = SharedDriverSteadyNowNs())
    {
        AxisRuntimeStatus status = EvaluateBase(feedback, command, intent, deviceHealth, nowNs);
        const bool healthyRecoverSample = status.axisReadyForEnable;
        if (intent == AxisIntent::Recover) {
            if (healthyRecoverSample) {
                if (feedback.lastRxSteadyNs != lastRecoverSampleNs_) {
                    lastRecoverSampleNs_ = feedback.lastRxSteadyNs;
                    if (lastStatus_.intent == AxisIntent::Recover && lastStatus_.axisReadyForEnable) {
                        ++recoverHealthyCycles_;
                    } else {
                        recoverHealthyCycles_ = 1;
                    }
                } else {
                    recoverHealthyCycles_ = std::max<std::uint32_t>(recoverHealthyCycles_, 1u);
                }
                if (recoverHealthyCycles_ < config_.recoverConfirmCycles) {
                    status.state = AxisRuntimeState::Recovering;
                } else {
                    status.recoverConfirmed = true;
                }
            } else {
                recoverHealthyCycles_ = 0;
                lastRecoverSampleNs_ = 0;
                status.state = AxisRuntimeState::Recovering;
                status.recoverConfirmed = false;
            }
        } else {
            recoverHealthyCycles_ = 0;
            lastRecoverSampleNs_ = 0;
            status.recoverConfirmed = status.axisReadyForEnable;
        }

        lastStatus_ = status;
        return status;
    }

    const AxisRuntimeStatus &lastStatus() const
    {
        return lastStatus_;
    }

    static bool ReadyForEnable(const AxisRuntimeStatus &status)
    {
        return status.axisReadyForEnable;
    }

    static bool ReadyForRun(const AxisRuntimeStatus &status)
    {
        return status.axisReadyForRun;
    }

    static bool RecoverConfirmed(const AxisRuntimeStatus &status)
    {
        return status.recoverConfirmed;
    }

    static bool MotionReady(const AxisRuntimeStatus &status)
    {
        return ReadyForRun(status);
    }

    static std::string DescribeBlockReason(const AxisRuntimeStatus &status)
    {
        if (!status.deviceReady || status.state == AxisRuntimeState::Offline) {
            return "Feedback offline.";
        }
        if (!status.feedbackReady) {
            return "Feedback degraded.";
        }
        if (!status.faultCleared) {
            return "Fault still active.";
        }
        if (!status.enabledReady) {
            return "Axis not enabled.";
        }
        if (!status.modeReady) {
            return "Mode not ready.";
        }
        if (status.state == AxisRuntimeState::Recovering) {
            return "Axis still recovering.";
        }
        return std::string();
    }

    static std::string DescribeMotionBlock(const AxisRuntimeStatus &status)
    {
        return DescribeBlockReason(status);
    }

private:
    AxisRuntimeStatus EvaluateBase(
        const SharedDriverState::AxisFeedbackState &feedback,
        const SharedDriverState::AxisCommandState *command,
        AxisIntent intent,
        const SharedDriverState::DeviceHealthState *deviceHealth,
        std::int64_t nowNs) const
    {
        AxisRuntimeStatus status;
        status.key = feedback.key;
        status.intent = intent;
        status.deviceReady = (deviceHealth == nullptr) ? true : deviceHealth->transportReady;
        status.feedbackSeen = feedback.feedbackSeen;
        status.degraded = feedback.degraded || feedback.consecutiveTimeoutCount > 0;
        status.fault = feedback.fault;
        status.enabled = feedback.enabled;
        status.commandValid = command != nullptr && command->valid;
        status.modeExpected = command != nullptr && command->desiredModeValid;

        status.feedbackFresh = status.feedbackSeen && feedback.lastRxSteadyNs > 0;
        if (status.feedbackFresh && config_.feedbackFreshnessTimeoutNs > 0 &&
            nowNs > feedback.lastRxSteadyNs) {
            status.feedbackFresh =
                (nowNs - feedback.lastRxSteadyNs) <= config_.feedbackFreshnessTimeoutNs;
        }

        if (status.modeExpected) {
            const bool timestampMatched =
                feedback.feedbackSeen && feedback.lastRxSteadyNs > 0 &&
                feedback.lastModeMatchSteadyNs >= feedback.lastRxSteadyNs;
            const bool valueMatched =
                feedback.feedbackSeen && command != nullptr &&
                feedback.mode == command->desiredMode;
            status.modeMatched = timestampMatched || valueMatched;
        }

        status.feedbackReady =
            status.deviceReady && status.feedbackSeen && status.feedbackFresh && !status.degraded;
        status.faultCleared = !status.fault;
        status.enabledReady = status.enabled;
        status.modeReady = !status.modeExpected || status.modeMatched;
        status.axisReadyForEnable = status.feedbackReady && status.faultCleared;
        status.axisReadyForRun =
            status.axisReadyForEnable && status.enabledReady && status.modeReady;

        if (!status.deviceReady || !status.feedbackSeen) {
            status.state = AxisRuntimeState::Offline;
            return status;
        }
        if (!status.feedbackReady) {
            status.state = AxisRuntimeState::Seen;
            return status;
        }
        if (!status.faultCleared) {
            status.state = AxisRuntimeState::Faulted;
            return status;
        }
        if (!status.enabledReady) {
            status.state = AxisRuntimeState::Standby;
            return status;
        }

        status.state = status.commandValid ? AxisRuntimeState::Running : AxisRuntimeState::Armed;
        return status;
    }

    Config config_{};
    AxisRuntimeStatus lastStatus_{};
    std::uint32_t recoverHealthyCycles_{0};
    std::int64_t lastRecoverSampleNs_{0};
};

} // namespace can_driver

#endif // CAN_DRIVER_AXIS_RUNTIME_H
