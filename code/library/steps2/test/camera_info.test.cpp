#include "steps/camera_info.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

using namespace reprojection;

class CameraInfoFixture : public ::testing::Test {
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
        auto const [image_loading_id, _]{db_.GetOrCreateStep(StepType::ImageLoading, "")};
        image_loading_id_ = image_loading_id;
        db_.ImagesInsert(image_loading_id_, camera_id_, *encoded_images);
    }

    database::CalibrationDatabase db_{database::CalibrationDatabase(":memory:", true)};
    RecordingId recording_id_{db_.GetOrCreateRecording("", "")};
    AssetId camera_id_{db_.GetOrCreateAsset(AssetType::Camera, 0, "")};
    StepId image_loading_id_{-1};
};

TEST_F(CameraInfoFixture, TestCameraInfoStepRunner) {
    auto const owner{steps::StepOwner::Recording(recording_id_)};

    steps::CameraInfoStep const step{camera_id_, image_loading_id_, CameraModel::DoubleSphere, db_};
    StepId const step_id{RunStep<steps::CameraInfoStep>(owner, step, db_)};

    auto const result{db_.CameraInfoSelect(step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->camera_model, CameraModel::DoubleSphere);
    EXPECT_EQ(result->bounds.u_max, 20);
    EXPECT_EQ(result->bounds.u_min, 0);
    EXPECT_EQ(result->bounds.v_max, 10);
    EXPECT_EQ(result->bounds.v_min, 0);
}

TEST_F(CameraInfoFixture, TestCameraInfoStep) {
    // Build the step and check that the type and hash function are correct.
    steps::CameraInfoStep const step{camera_id_, image_loading_id_, CameraModel::DoubleSphere, db_};
    EXPECT_EQ(step.Type(), StepType::CameraInfo);
    EXPECT_EQ(step.CacheKey().value, "9b077a5c721520f1cbee0879eb92229c61e722ec9f0a79e3d750759785db8247");

    // Build the actual database step id and execute the step.
    auto const [step_id, _]{db_.GetOrCreateStep(StepType::CameraInfo, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{db_.CameraInfoSelect(step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->camera_model, CameraModel::DoubleSphere);
    EXPECT_EQ(result->bounds.u_max, 20);
    EXPECT_EQ(result->bounds.u_min, 0);
    EXPECT_EQ(result->bounds.v_max, 10);
    EXPECT_EQ(result->bounds.v_min, 0);
}
