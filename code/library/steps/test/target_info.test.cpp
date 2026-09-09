#include "steps/target_info.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class TargetInfoTestFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        target_id_ = context_.assets.target.id;
    }

    AssetId target_id_;
};

TEST_F(TargetInfoTestFixture, TestTargetInfoStepRunner) {
    // Execute step.
    steps::TargetInfoStep const step{target_id_, context_.assets.target.config};
    StepId const step_id{RunStep<steps::TargetInfoStep>(context_.workflow_id, step, db_)};

    // Compare reloaded artifact against original artifact.
    auto const result{database::TargetInfoSelect(db_.get(), step_id, target_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(result->height, 5);
    EXPECT_EQ(result->width, 4);
}

TEST_F(TargetInfoTestFixture, TestTargetInfoStep) {
    steps::TargetInfoStep const step{target_id_, context_.assets.target.config};
    EXPECT_EQ(step.Type(), StepType::TargetInfo);
    EXPECT_EQ(step.Assets(), std::vector{target_id_});
    EXPECT_EQ(step.CacheKey().value, "ac951183ef1cc7b2c5944340306e7901fe75b09d89777999761d4c8713a1ca4d");

    auto const [step_id, _]{database::GetOrCreateStep(db_.get(), StepType::TargetInfo, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::TargetInfoSelect(db_.get(), step_id, target_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(result->height, 5);
    EXPECT_EQ(result->width, 4);
}