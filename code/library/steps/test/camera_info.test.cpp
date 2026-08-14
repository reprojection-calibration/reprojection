#include "steps/camera_info.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class CameraInfoTestFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        // TODO(Jack): This block of code is copy and pasted across multiple fixtures!
        // Build the encoded images (cv::Mat -> serialized buffer)
        cv::Mat const img{cv::Mat::zeros(10, 20, CV_8UC1)};
        std::vector<uchar> buffer;
        if (not cv::imencode(".png", img, buffer)) {
            throw std::runtime_error("cv::imencode() failed");
        }
        EncodedImages const encoded_images{{{1, ImageBuffer{buffer}}, {2, ImageBuffer{buffer}}}};
        image_loading_id_ = InsertImages(encoded_images);
    }

    StepId image_loading_id_;
};

TEST_F(CameraInfoTestFixture, TestCameraInfoStepRunner) {
    steps::CameraInfoStep const step{camera_id_, image_loading_id_, CameraModel::DoubleSphere, db_};
    StepId const step_id{RunStep<steps::CameraInfoStep>(workflow_id_, step, db_)};

    auto const result{database::CameraInfoSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->camera_model, CameraModel::DoubleSphere);
    EXPECT_EQ(result->bounds.u_max, 20);
    EXPECT_EQ(result->bounds.u_min, 0);
    EXPECT_EQ(result->bounds.v_max, 10);
    EXPECT_EQ(result->bounds.v_min, 0);
}

TEST_F(CameraInfoTestFixture, TestCameraInfoStep) {
    // Build the step and check that the type and hash function are correct.
    steps::CameraInfoStep const step{camera_id_, image_loading_id_, CameraModel::DoubleSphere, db_};
    EXPECT_EQ(step.Type(), StepType::CameraInfo);
    EXPECT_EQ(step.CacheKey().value, "9a845e2b13b28d7676c58c24d20b68ce5b427d855d0fc06b136a682e881ed75f");

    // Build the actual database step id and execute the step.
    auto const [step_id, _]{database::GetOrCreateStep(db_.get(), StepType::CameraInfo, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::CameraInfoSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->camera_model, CameraModel::DoubleSphere);
    EXPECT_EQ(result->bounds.u_max, 20);
    EXPECT_EQ(result->bounds.u_min, 0);
    EXPECT_EQ(result->bounds.v_max, 10);
    EXPECT_EQ(result->bounds.v_min, 0);
}
