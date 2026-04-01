#ifndef CAN_DRIVER_REFRESH_SCHEDULER_H
#define CAN_DRIVER_REFRESH_SCHEDULER_H

#include "can_driver/CanProtocol.h"
#include "can_driver/SharedDriverState.h"

#include <chrono>
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

constexpr std::size_t kPpRefreshQueryCount = 6;

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

struct PpAxisRefreshScheduleState {
    std::array<std::chrono::steady_clock::time_point, kPpRefreshQueryCount> lastScheduled {};
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

inline constexpr std::array<PpRefreshQuery, kPpRefreshQueryCount> PpRefreshQueryOrder()
{
    return {PpRefreshQuery::Position,
            PpRefreshQuery::Velocity,
            PpRefreshQuery::Mode,
            PpRefreshQuery::Enable,
            PpRefreshQuery::Fault,
            PpRefreshQuery::Current};
}

inline constexpr std::size_t PpRefreshQueryIndex(PpRefreshQuery query)
{
    switch (query) {
    case PpRefreshQuery::Position:
        return 0;
    case PpRefreshQuery::Velocity:
        return 1;
    case PpRefreshQuery::Mode:
        return 2;
    case PpRefreshQuery::Enable:
        return 3;
    case PpRefreshQuery::Fault:
        return 4;
    case PpRefreshQuery::Current:
        return 5;
    }
    return 0;
}

inline std::chrono::milliseconds PpRefreshPeriod(PpRefreshQuery query,
                                                 bool queryPressureActive,
                                                 const PpAxisRefreshSnapshot &snapshot)
{
    const bool priority = NeedsPpPriorityLifecycleQueries(snapshot);
    switch (query) {
    case PpRefreshQuery::Position:
        return std::chrono::milliseconds(50);
    case PpRefreshQuery::Velocity:
        return priority ? std::chrono::milliseconds(50) : std::chrono::milliseconds(100);
    case PpRefreshQuery::Mode:
    case PpRefreshQuery::Enable:
    case PpRefreshQuery::Fault:
        return priority ? std::chrono::milliseconds(50) : std::chrono::milliseconds(100);
    case PpRefreshQuery::Current:
        // 压力模式后续改成“限预算不饿死字段”。当前先保证 freshness 上限。
        (void)queryPressureActive;
        return std::chrono::milliseconds(100);
    }
    return std::chrono::milliseconds(100);
}

inline PpRefreshPlan BuildPpRefreshPlan(std::chrono::steady_clock::time_point now,
                                        bool queryPressureActive,
                                        const PpAxisRefreshSnapshot &snapshot,
                                        PpAxisRefreshScheduleState *state)
{
    PpRefreshPlan plan;
    if (!state) {
        return plan;
    }

    for (const auto query : PpRefreshQueryOrder()) {
        const auto period = PpRefreshPeriod(query, queryPressureActive, snapshot);
        if (period <= std::chrono::milliseconds::zero()) {
            continue;
        }
        const auto index = PpRefreshQueryIndex(query);
        const auto lastScheduled = state->lastScheduled[index];
        if (lastScheduled != std::chrono::steady_clock::time_point {} &&
            now >= lastScheduled &&
            (now - lastScheduled) < period) {
            continue;
        }
        plan.push(query);
        state->lastScheduled[index] = now;
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
