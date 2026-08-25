#include "steps/feature_extraction.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class FeatureExtractionTestFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        // TODO(Jack): This block of code is copy and pasted across multiple fixtures!
        cv::Mat const img{cv::Mat::zeros(10, 20, CV_8UC1)};
        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            throw std::runtime_error("cv::imencode() failed");
        }
        EncodedImages const encoded_images{{{1, ImageBuffer{buffer}}, {2, ImageBuffer{buffer}}}};
        image_loading_id_ = InsertImages(encoded_images);

        target_info_id_ = database::GetOrCreateStep(db_.get(), StepType::TargetInfo, "").first;
        TargetInfo const target_info{TargetType::Aprilgrid3, 6, 8, 0.1, false};
        database::TargetInfoInsert(db_.get(), target_info_id_, target_id_, target_info);
    }

    StepId image_loading_id_;
    StepId target_info_id_;
    AssetId target_id_{database::GetOrCreateAsset(db_.get(), AssetType::Target, 0, "")};
};

TEST_F(FeatureExtractionTestFixture, TestFeatureExtractionStepRunner) {
    steps::FeatureExtraction const step{camera_id_, image_loading_id_, false, target_info_id_, target_id_, db_};
    StepId const step_id{RunStep<steps::FeatureExtraction>(workflow_id_, step, db_)};

    // TODO(Jack): This is kind of an anti climatic result but it's not our responsibility to check that the feature
    // extraction works here.
    auto const result{database::ExtractedTargetsSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 0);
}

TEST_F(FeatureExtractionTestFixture, TestFeatureExtractionStep) {
    // Build the step and check that the type and hash function are correct.
    steps::FeatureExtraction const step{camera_id_, image_loading_id_, false, target_info_id_, target_id_, db_};
    EXPECT_EQ(step.Type(), StepType::FeatureExtraction);
    EXPECT_EQ(step.CacheKey().value, "cebb7a03270d515c4a1fe8be46ce42e546b3958e655f3af29197303d055e5b1a");

    // Build the actual database step id and execute the step.
    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::FeatureExtraction, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::ExtractedTargetsSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 0);
}