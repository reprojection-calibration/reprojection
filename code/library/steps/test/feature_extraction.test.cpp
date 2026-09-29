#include "steps/feature_extraction.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

// TODO(Jack): We are now two levels of test fixture inheritance deep! Be careful and tread lightly :)
class FeatureExtractionTestFixture : public ImageSamplerFixture {
   protected:
    void SetUp() override {
        ImageSamplerFixture::SetUp();

        target_id_ = context_.assets.target.id;
        target_info_id_ = InsertTargetInfo();
    }

    AssetId target_id_;
    StepId target_info_id_;
};

TEST_F(FeatureExtractionTestFixture, TestFeatureExtractionStepRunner) {
    steps::FeatureExtraction const step{camera_id_, "", image_sampler_, false, target_info_id_, target_id_, db_};
    StepId const step_id{RunStep<steps::FeatureExtraction>(context_.workflow_id, step, db_)};

    // TODO(Jack): This is kind of an anti climatic result but it's not our responsibility to check that the feature
    // extraction works here.
    auto const result{database::TargetsSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 0);
}

TEST_F(FeatureExtractionTestFixture, TestFeatureExtractionStep) {
    // Build the step and check that the type and hash function are correct.
    steps::FeatureExtraction const step{camera_id_, "", image_sampler_, false, target_info_id_, target_id_, db_};
    EXPECT_EQ(step.Type(), StepType::FeatureExtraction);
    EXPECT_EQ(step.Assets(), std::vector{camera_id_});
    EXPECT_EQ(step.CacheKey().value, "7b5875b215c1119d367225c1c3d3d3019dd87acd6d7164603f77d423a9e84954");

    // Build the actual database step id and execute the step.
    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::FeatureExtraction, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::TargetsSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 0);
}