#include "steps/visual_inertial_init.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"
#include "types/physics_constants.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class VisualInertialInitFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        cam_id_ = context_.assets.cameras.front().id;
        imu_id_ = context_.assets.imu->id;  // Unprotected optional access!
        std::tie(imu_data_id_, spline_id_) = InsertImuSetup(cam_id_);
    }

    AssetId cam_id_;
    AssetId imu_id_;
    StepId imu_data_id_;
    StepId spline_id_;
};

TEST_F(VisualInertialInitFixture, TestExtrinsicInitStepRunner) {
    steps::VisualInertialInit const step{imu_id_, imu_data_id_, cam_id_, spline_id_, 1, db_};
    StepId const step_id{RunStep<steps::VisualInertialInit>(context_.workflow_id, step, db_)};

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, imu_id_, cam_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);  // Heuristic!

    auto const result2{database::GravitySelect(db_.get(), step_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_NEAR(result2->norm(), kGravity, 1e-3);  // Heuristic!
}

TEST_F(VisualInertialInitFixture, TestExtrinsicInitStep) {
    steps::VisualInertialInit const step{imu_id_, imu_data_id_, cam_id_, spline_id_, 1, db_};

    EXPECT_EQ(step.Type(), StepType::ExtrinsicInit);
    std::vector const gt_assets{imu_id_, cam_id_};
    EXPECT_EQ(step.Assets(), gt_assets);
    EXPECT_EQ(step.CacheKey().value, "d78f7d0b3bf9ef156ed4b8c9c31eaf1fcefb3174b239d1b5e471de80c488bc05");

    // Build the actual database step id and execute the step.
    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::ExtrinsicInit, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, imu_id_, cam_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);

    auto const result2{database::GravitySelect(db_.get(), step_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_NEAR(result2->norm(), kGravity, 1e-3);
}