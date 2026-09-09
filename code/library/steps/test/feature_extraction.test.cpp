#include "steps/feature_extraction.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class FeatureExtractionTestFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        camera_id_ = context_.assets.cameras.front().id;
        target_id_ = context_.assets.target.id;
        image_loading_id_ = InsertImages(camera_id_, TestImageSamples());
        target_info_id_ = InsertTargetInfo();
    }

    AssetId camera_id_;
    AssetId target_id_;
    StepId image_loading_id_;
    StepId target_info_id_;
};

TEST_F(FeatureExtractionTestFixture, TestFeatureExtractionStepRunner) {
    steps::FeatureExtraction const step{camera_id_, image_loading_id_, false, target_info_id_, target_id_, db_};
    StepId const step_id{RunStep<steps::FeatureExtraction>(context_.workflow_id, step, db_)};

    // TODO(Jack): This is kind of an anti climatic result but it's not our responsibility to check that the feature
    // extraction works here.
    auto const result{database::TargetsSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 0);
}

TEST_F(FeatureExtractionTestFixture, TestFeatureExtractionStep) {
    // Build the step and check that the type and hash function are correct.
    steps::FeatureExtraction const step{camera_id_, image_loading_id_, false, target_info_id_, target_id_, db_};
    EXPECT_EQ(step.Type(), StepType::FeatureExtraction);
    EXPECT_EQ(step.Assets(), std::vector{camera_id_});
    EXPECT_EQ(step.CacheKey().value, "efde0415f6bbd3b38ab818c62ef0c95208624f9867445c29e60dac88c1b81d5a");

    // Build the actual database step id and execute the step.
    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::FeatureExtraction, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::TargetsSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 0);
}