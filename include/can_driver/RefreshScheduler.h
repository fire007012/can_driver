#ifndef CAN_DRIVER_REFRESH_SCHEDULER_H
#define CAN_DRIVER_REFRESH_SCHEDULER_H

#include "can_driver/CanProtocol.h"
#include "can_driver/SharedDriverState.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace can_driver {

enum class PpRefreshQuery : std::uint8_t {
    Position,
    Velocity,
    Mode,
    Enable,
    Fault,
    Current,
};

enum class MtRefreshQuery : std::uint8_t {
    State,
    MultiTurnAngle,
    Error,
};

template <typename Query, std::size_t Capacity>
struct RefreshPlan {
    std::array<Query, Capacity> items{};
    std::size_t count{0};

    void push(Query query)
    {
        if (count < Capacity) {
            items[count++] = query;
        }
    }
};

using PpRefreshPlan = RefreshPlan<PpRefreshQuery, 6>;
using MtRefreshPlan = RefreshPlan<MtRefreshQuery, 3>;

struct PpAxisRefreshSnapshot {
    bool feedbackSeen{false};
    bool enabled{false};
    bool fault{false};
    bool degraded{false};
    bool desiredModeValid{false};
    CanProtocol::MotorMode feedbackMode{CanProtocol::MotorMode::Position};
    CanProtocol::MotorMode desiredMode{CanProtocol::MotorMode::Position};
    AxisIntent intent{AxisIntent::None};
};

inline bool NeedsPpPriorityLifecycleQueries(const PpAxisRefreshSnapshot &snapshot)
{
    if (!snapshot.feedbackSeen || snapshot.degraded || snapshot.fault || !snapshot.enabled) {
        return true;
    }
    if (snapshot.intent == AxisIntent::Enable || snapshot.intent == AxisIntent::Recover) {
        return true;
    }
    if (snapshot.desiredModeValid && snapshot.feedbackMode != snapshot.desiredMode) {
        return true;
    }
    return false;
}

inline PpRefreshPlan BuildPpRefreshPlan(std::uint64_t cycle,
                                        std::size_t motorIndex,
                                        bool queryPressureActive,
                                        const PpAxisRefreshSnapshot &snapshot)
{
    PpRefreshPlan plan;
    if (queryPressureActive) {
        if (((cycle + motorIndex) % 2) == 0) {
            plan.push(PpRefreshQuery::Position);
        } else {
            plan.push(PpRefreshQuery::Velocity);
        }

        switch (static_cast<std::size_t>((cycle + motorIndex) % 3)) {
        case 0:
            plan.push(PpRefreshQuery::Mode);
            break;
        case 1:
            plan.push(PpRefreshQuery::Enable);
            break;
        case 2:
        default:
            plan.push(PpRefreshQuery::Fault);
            break;
        }
        return plan;
    }

    plan.push(PpRefreshQuery::Position);
    if (NeedsPpPriorityLifecycleQueries(snapshot)) {
        plan.push(PpRefreshQuery::Velocity);
        plan.push(PpRefreshQuery::Mode);
        plan.push(PpRefreshQuery::Enable);
        plan.push(PpRefreshQuery::Fault);
        if (((cycle + motorIndex) % 4) == 0) {
            plan.push(PpRefreshQuery::Current);
        }
        return plan;
    }

    if (((cycle + motorIndex) % 2) == 0) {
        plan.push(PpRefreshQuery::Velocity);
    }

    switch (static_cast<std::size_t>((cycle + motorIndex) % 8)) {
    case 0:
        plan.push(PpRefreshQuery::Mode);
        break;
    case 2:
        plan.push(PpRefreshQuery::Enable);
        break;
    case 4:
        plan.push(PpRefreshQuery::Fault);
        break;
    case 6:
    default:
        plan.push(PpRefreshQuery::Current);
        break;
    case 1:
    case 3:
    case 5:
    case 7:
        break;
    }
    return plan;
}

inline MtRefreshPlan BuildMtRefreshPlan(std::uint64_t cycle,
                                        std::size_t motorIndex,
                                        bool queryPressureActive)
{
    MtRefreshPlan plan;
    if (queryPressureActive) {
        if (((cycle + motorIndex) % 2) == 0) {
            plan.push(MtRefreshQuery::State);
        } else {
            plan.push(MtRefreshQuery::MultiTurnAngle);
        }
        plan.push(MtRefreshQuery::Error);
        return plan;
    }

    plan.push(MtRefreshQuery::State);
    plan.push(MtRefreshQuery::MultiTurnAngle);
    plan.push(MtRefreshQuery::Error);
    return plan;
}

} // namespace can_driver

#endif // CAN_DRIVER_REFRESH_SCHEDULER_H
