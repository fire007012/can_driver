#include <gtest/gtest.h>

#include "can_driver/RefreshScheduler.h"

namespace {

TEST(RefreshSchedulerTest, PpHealthyPlanRotatesLifecycleSampling)
{
    can_driver::PpAxisRefreshSnapshot snapshot;
    snapshot.feedbackSeen = true;
    snapshot.enabled = true;
    snapshot.desiredModeValid = true;
    snapshot.feedbackMode = CanProtocol::MotorMode::Position;
    snapshot.desiredMode = CanProtocol::MotorMode::Position;
    snapshot.intent = can_driver::AxisIntent::Run;

    const auto plan0 = can_driver::BuildPpRefreshPlan(0, 0, false, snapshot);
    ASSERT_EQ(plan0.count, 3u);
    EXPECT_EQ(plan0.items[0], can_driver::PpRefreshQuery::Position);
    EXPECT_EQ(plan0.items[1], can_driver::PpRefreshQuery::Velocity);
    EXPECT_EQ(plan0.items[2], can_driver::PpRefreshQuery::Mode);

    const auto plan1 = can_driver::BuildPpRefreshPlan(1, 0, false, snapshot);
    ASSERT_EQ(plan1.count, 1u);
    EXPECT_EQ(plan1.items[0], can_driver::PpRefreshQuery::Position);

    const auto plan2 = can_driver::BuildPpRefreshPlan(2, 0, false, snapshot);
    ASSERT_EQ(plan2.count, 3u);
    EXPECT_EQ(plan2.items[2], can_driver::PpRefreshQuery::Enable);

    const auto plan3 = can_driver::BuildPpRefreshPlan(3, 0, false, snapshot);
    ASSERT_EQ(plan3.count, 1u);
    EXPECT_EQ(plan3.items[0], can_driver::PpRefreshQuery::Position);

    const auto plan4 = can_driver::BuildPpRefreshPlan(4, 0, false, snapshot);
    ASSERT_EQ(plan4.count, 3u);
    EXPECT_EQ(plan4.items[2], can_driver::PpRefreshQuery::Fault);

    const auto plan6 = can_driver::BuildPpRefreshPlan(6, 0, false, snapshot);
    ASSERT_EQ(plan6.count, 3u);
    EXPECT_EQ(plan6.items[2], can_driver::PpRefreshQuery::Current);
}

TEST(RefreshSchedulerTest, PpPriorityPlanKeepsLifecycleQueriesAndSamplesCurrent)
{
    can_driver::PpAxisRefreshSnapshot snapshot;
    snapshot.feedbackSeen = false;

    const auto plan0 = can_driver::BuildPpRefreshPlan(0, 0, false, snapshot);
    ASSERT_EQ(plan0.count, 6u);
    EXPECT_EQ(plan0.items[0], can_driver::PpRefreshQuery::Position);
    EXPECT_EQ(plan0.items[1], can_driver::PpRefreshQuery::Velocity);
    EXPECT_EQ(plan0.items[2], can_driver::PpRefreshQuery::Mode);
    EXPECT_EQ(plan0.items[3], can_driver::PpRefreshQuery::Enable);
    EXPECT_EQ(plan0.items[4], can_driver::PpRefreshQuery::Fault);
    EXPECT_EQ(plan0.items[5], can_driver::PpRefreshQuery::Current);

    const auto plan1 = can_driver::BuildPpRefreshPlan(1, 0, false, snapshot);
    ASSERT_EQ(plan1.count, 5u);
    EXPECT_EQ(plan1.items[0], can_driver::PpRefreshQuery::Position);
    EXPECT_EQ(plan1.items[1], can_driver::PpRefreshQuery::Velocity);
    EXPECT_EQ(plan1.items[2], can_driver::PpRefreshQuery::Mode);
    EXPECT_EQ(plan1.items[3], can_driver::PpRefreshQuery::Enable);
    EXPECT_EQ(plan1.items[4], can_driver::PpRefreshQuery::Fault);
}

TEST(RefreshSchedulerTest, PpPressurePlanAlternatesFastFeedbackAndLifecycleQueries)
{
    can_driver::PpAxisRefreshSnapshot snapshot;

    const auto plan0 = can_driver::BuildPpRefreshPlan(0, 0, true, snapshot);
    ASSERT_EQ(plan0.count, 2u);
    EXPECT_EQ(plan0.items[0], can_driver::PpRefreshQuery::Position);
    EXPECT_EQ(plan0.items[1], can_driver::PpRefreshQuery::Mode);

    const auto plan1 = can_driver::BuildPpRefreshPlan(1, 0, true, snapshot);
    ASSERT_EQ(plan1.count, 2u);
    EXPECT_EQ(plan1.items[0], can_driver::PpRefreshQuery::Velocity);
    EXPECT_EQ(plan1.items[1], can_driver::PpRefreshQuery::Enable);

    const auto plan2 = can_driver::BuildPpRefreshPlan(2, 0, true, snapshot);
    ASSERT_EQ(plan2.count, 2u);
    EXPECT_EQ(plan2.items[0], can_driver::PpRefreshQuery::Position);
    EXPECT_EQ(plan2.items[1], can_driver::PpRefreshQuery::Fault);
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
