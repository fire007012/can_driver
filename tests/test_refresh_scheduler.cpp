#include <gtest/gtest.h>

#include "can_driver/RefreshScheduler.h"

namespace {

using steady_tp = std::chrono::steady_clock::time_point;

bool planContains(const can_driver::PpRefreshPlan &plan, can_driver::PpRefreshQuery query)
{
    for (std::size_t i = 0; i < plan.count; ++i) {
        if (plan.items[i] == query) {
            return true;
        }
    }
    return false;
}

void noteIssued(steady_tp now,
                can_driver::PpAxisRefreshScheduleState *state,
                const can_driver::PpRefreshPlan &plan)
{
    for (std::size_t i = 0; i < plan.count; ++i) {
        can_driver::NotePpRefreshQueryDue(now, state, plan.items[i]);
        can_driver::NotePpRefreshQueryIssued(now, state, plan.items[i]);
    }
}

TEST(RefreshSchedulerTest, PpHealthyPlanKeepsAllFieldsWithinDeadline)
{
    can_driver::PpAxisRefreshSnapshot snapshot;
    snapshot.feedbackSeen = true;
    snapshot.enabled = true;
    snapshot.desiredModeValid = true;
    snapshot.feedbackMode = CanProtocol::MotorMode::Position;
    snapshot.desiredMode = CanProtocol::MotorMode::Position;
    snapshot.intent = can_driver::AxisIntent::Run;

    can_driver::PpAxisRefreshScheduleState state;
    const auto t0 = steady_tp {} + std::chrono::milliseconds(1);

    const auto plan0 = can_driver::BuildPpRefreshPlan(t0, false, snapshot, &state);
    EXPECT_EQ(plan0.count, can_driver::kPpRefreshQueryCount);
    EXPECT_TRUE(planContains(plan0, can_driver::PpRefreshQuery::Position));
    EXPECT_TRUE(planContains(plan0, can_driver::PpRefreshQuery::Velocity));
    EXPECT_TRUE(planContains(plan0, can_driver::PpRefreshQuery::Mode));
    EXPECT_TRUE(planContains(plan0, can_driver::PpRefreshQuery::Enable));
    EXPECT_TRUE(planContains(plan0, can_driver::PpRefreshQuery::Fault));
    EXPECT_TRUE(planContains(plan0, can_driver::PpRefreshQuery::Current));
    noteIssued(t0, &state, plan0);

    const auto plan1 =
        can_driver::BuildPpRefreshPlan(t0 + std::chrono::milliseconds(50), false, snapshot, &state);
    ASSERT_EQ(plan1.count, 1u);
    EXPECT_EQ(plan1.items[0], can_driver::PpRefreshQuery::Position);
    noteIssued(t0 + std::chrono::milliseconds(50), &state, plan1);

    const auto plan2 =
        can_driver::BuildPpRefreshPlan(t0 + std::chrono::milliseconds(100), false, snapshot, &state);
    EXPECT_EQ(plan2.count, can_driver::kPpRefreshQueryCount);
    EXPECT_TRUE(planContains(plan2, can_driver::PpRefreshQuery::Current));
    noteIssued(t0 + std::chrono::milliseconds(100), &state, plan2);

    const auto plan3 =
        can_driver::BuildPpRefreshPlan(t0 + std::chrono::milliseconds(150), false, snapshot, &state);
    ASSERT_EQ(plan3.count, 1u);
    EXPECT_EQ(plan3.items[0], can_driver::PpRefreshQuery::Position);
}

TEST(RefreshSchedulerTest, PpPriorityPlanAcceleratesLifecycleQueriesTo50Ms)
{
    can_driver::PpAxisRefreshSnapshot snapshot;
    snapshot.feedbackSeen = false;
    can_driver::PpAxisRefreshScheduleState state;
    const auto t0 = steady_tp {} + std::chrono::milliseconds(1);

    const auto plan0 = can_driver::BuildPpRefreshPlan(t0, false, snapshot, &state);
    ASSERT_EQ(plan0.count, 6u);
    EXPECT_TRUE(planContains(plan0, can_driver::PpRefreshQuery::Current));
    noteIssued(t0, &state, plan0);

    const auto plan1 =
        can_driver::BuildPpRefreshPlan(t0 + std::chrono::milliseconds(50), false, snapshot, &state);
    ASSERT_EQ(plan1.count, 5u);
    EXPECT_TRUE(planContains(plan1, can_driver::PpRefreshQuery::Position));
    EXPECT_TRUE(planContains(plan1, can_driver::PpRefreshQuery::Velocity));
    EXPECT_TRUE(planContains(plan1, can_driver::PpRefreshQuery::Mode));
    EXPECT_TRUE(planContains(plan1, can_driver::PpRefreshQuery::Enable));
    EXPECT_TRUE(planContains(plan1, can_driver::PpRefreshQuery::Fault));
    EXPECT_FALSE(planContains(plan1, can_driver::PpRefreshQuery::Current));
    noteIssued(t0 + std::chrono::milliseconds(50), &state, plan1);
}

TEST(RefreshSchedulerTest, PpPressurePlanDoesNotStarveCurrentBeyond100Ms)
{
    can_driver::PpAxisRefreshSnapshot snapshot;
    snapshot.feedbackSeen = true;
    snapshot.enabled = true;
    snapshot.desiredModeValid = true;
    snapshot.feedbackMode = CanProtocol::MotorMode::Position;
    snapshot.desiredMode = CanProtocol::MotorMode::Position;
    snapshot.intent = can_driver::AxisIntent::Run;
    can_driver::PpAxisRefreshScheduleState state;
    const auto t0 = steady_tp {} + std::chrono::milliseconds(1);

    const auto plan0 = can_driver::BuildPpRefreshPlan(t0, true, snapshot, &state);
    ASSERT_EQ(plan0.count, 6u);
    EXPECT_TRUE(planContains(plan0, can_driver::PpRefreshQuery::Current));
    noteIssued(t0, &state, plan0);

    const auto plan1 =
        can_driver::BuildPpRefreshPlan(t0 + std::chrono::milliseconds(50), true, snapshot, &state);
    ASSERT_EQ(plan1.count, 1u);
    EXPECT_EQ(plan1.items[0], can_driver::PpRefreshQuery::Position);
    noteIssued(t0 + std::chrono::milliseconds(50), &state, plan1);

    const auto plan2 =
        can_driver::BuildPpRefreshPlan(t0 + std::chrono::milliseconds(100), true, snapshot, &state);
    EXPECT_TRUE(planContains(plan2, can_driver::PpRefreshQuery::Current));
}

TEST(RefreshSchedulerTest, PpPriorityQueryDetectionCoversRecoverAndModeMismatch)
{
    can_driver::PpAxisRefreshSnapshot recoverSnapshot;
    recoverSnapshot.feedbackSeen = true;
    recoverSnapshot.enabled = true;
    recoverSnapshot.intent = can_driver::AxisIntent::Recover;
    EXPECT_TRUE(can_driver::NeedsPpPriorityLifecycleQueries(recoverSnapshot));

    can_driver::PpAxisRefreshSnapshot mismatchSnapshot;
    mismatchSnapshot.feedbackSeen = true;
    mismatchSnapshot.enabled = true;
    mismatchSnapshot.desiredModeValid = true;
    mismatchSnapshot.feedbackMode = CanProtocol::MotorMode::Velocity;
    mismatchSnapshot.desiredMode = CanProtocol::MotorMode::CSP;
    EXPECT_TRUE(can_driver::NeedsPpPriorityLifecycleQueries(mismatchSnapshot));
}

TEST(RefreshSchedulerTest, MtPressurePlanKeepsErrorSamplingWhileAlternatingFeedback)
{
    const auto plan0 = can_driver::BuildMtRefreshPlan(0, 0, true);
    ASSERT_EQ(plan0.count, 2u);
    EXPECT_EQ(plan0.items[0], can_driver::MtRefreshQuery::State);
    EXPECT_EQ(plan0.items[1], can_driver::MtRefreshQuery::Error);

    const auto plan1 = can_driver::BuildMtRefreshPlan(1, 0, true);
    ASSERT_EQ(plan1.count, 2u);
    EXPECT_EQ(plan1.items[0], can_driver::MtRefreshQuery::MultiTurnAngle);
    EXPECT_EQ(plan1.items[1], can_driver::MtRefreshQuery::Error);

    const auto steadyPlan = can_driver::BuildMtRefreshPlan(2, 0, false);
    ASSERT_EQ(steadyPlan.count, 3u);
    EXPECT_EQ(steadyPlan.items[0], can_driver::MtRefreshQuery::State);
    EXPECT_EQ(steadyPlan.items[1], can_driver::MtRefreshQuery::MultiTurnAngle);
    EXPECT_EQ(steadyPlan.items[2], can_driver::MtRefreshQuery::Error);
}

} // namespace
