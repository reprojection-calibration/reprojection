#include "steps/spline_init.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class SplineInitFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        auto const& id{context_.assets.cameras.front().id};
        auto const targets_id{InsertExtractedTargets(id)};

        cam_ = CamStageIds{id, InsertCameraInfo(id), targets_id, InsertPoses(id, targets_id), InsertIntrinsic(id)};
    }

    CamStageIds cam_;
};

TEST_F(SplineInitFixture, TestSplineInitStepRunner) {
    steps::SplineInit const step{cam_, db_};
    StepId const step_id{RunStep<steps::SplineInit>(context_.workflow_id, step, db_)};

    auto const result{database::ControlPointsSelect(db_.get(), step_id, cam_.asset_id)};
    EXPECT_EQ(std::size(result), 3978);  // Heuristic

    auto const result2{database::SplineInfoSelect(db_.get(), step_id, cam_.asset_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_EQ(result2->t0_ns_, 2200000000);     // Heuristic
    EXPECT_EQ(result2->delta_t_ns_, 10000000);  // Heuristic
}

TEST_F(SplineInitFixture, TestSplineInitStep) {
    steps::SplineInit const step{cam_, db_};
    EXPECT_EQ(step.Type(), StepType::SplineInit);
    EXPECT_EQ(step.Assets(), std::vector{cam_.asset_id});
    EXPECT_EQ(step.CacheKey().value, "46d20a41437bc2c70b8497e5d0cebef0fcfb8bed7854b6e4ac1f1664ef006d02");

    auto const [step_id, _]{database::GetOrCreateStep(db_.get(), StepType::SplineInit, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::ControlPointsSelect(db_.get(), step_id, cam_.asset_id)};
    EXPECT_EQ(std::size(result), 3978);

    auto const result2{database::SplineInfoSelect(db_.get(), step_id, cam_.asset_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_EQ(result2->t0_ns_, 2200000000);
    EXPECT_EQ(result2->delta_t_ns_, 10000000);
}
