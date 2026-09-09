#include "steps/camera_info.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class CameraInfoTestFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        camera_id_ = context_.assets.cameras.front().id;
        image_loading_id_ = InsertImages(camera_id_, TestImageSamples());
    }

    AssetId camera_id_;
    StepId image_loading_id_;
};

TEST_F(CameraInfoTestFixture, TestCameraInfoStepRunner) {
    steps::CameraInfoStep const step{camera_id_, image_loading_id_, CameraModel::DoubleSphere, db_};
    StepId const step_id{RunStep<steps::CameraInfoStep>(context_.workflow_id, step, db_)};

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
    EXPECT_EQ(step.Assets(), std::vector{camera_id_});
    EXPECT_EQ(step.CacheKey().value, "a280620d0fa47563aa82125715dc11f22fa92b56ccff7bf90216a50ad396cbea");

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
