#include "steps/feature_extraction.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

using namespace reprojection;

class FeatureExtractionFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        // Build the encoded images (cv::Mat -> serialized buffer)
        cv::Mat const img{cv::Mat::zeros(10, 20, CV_8UC1)};
        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            throw std::runtime_error("cv::imencode() failed");
        }
        std::shared_ptr const encoded_images{
            std::make_shared<EncodedImages>(EncodedImages{{1, ImageBuffer{buffer}}, {2, ImageBuffer{buffer}}})};

        // Insert the images into the database.
        auto const [image_loading_id, _]{db_.GetOrCreateStep(recording_id_, std::nullopt, StepType::ImageLoading, "")};
        image_loading_id_ = image_loading_id;
        db_.ImagesInsert(image_loading_id_, camera_id_, *encoded_images);

        // Create the target and insert it into the database.
        TargetInfo const target_info{TargetType::Aprilgrid3, 6, 8, 0.1, false};
        auto const [target_info_id, _1]{db_.GetOrCreateStep(recording_id_, std::nullopt, StepType::TargetInfo, "")};
        target_info_id_ = target_info_id;
        db_.TargetInfoInsert(target_info_id, target_id_, target_info);
    }

    database::CalibrationDatabase db_{database::CalibrationDatabase(":memory:", true)};
    RecordingId recording_id_{db_.GetOrCreateRecording("", "")};
    AssetId camera_id_{db_.GetOrCreateAsset(AssetType::Camera, 0, "")};
    StepId image_loading_id_{-1};
    AssetId target_id_{db_.GetOrCreateAsset(AssetType::Target, 0, "")};
    StepId target_info_id_{-1};
};

TEST_F(FeatureExtractionFixture, TestFeatureExtractionStepRunner) {
    auto const owner{steps::StepOwner::Recording(recording_id_)};

    steps::FeatureExtraction const step{camera_id_, image_loading_id_, false, target_info_id_, target_id_, db_};

    StepId const step_id{RunStep<steps::FeatureExtraction>(owner, step, db_)};

    auto const result{db_.ExtractedTargetsSelect(step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 0);
}
