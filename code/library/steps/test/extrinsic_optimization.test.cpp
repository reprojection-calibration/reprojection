#include "steps/extrinsic_optimization.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"
#include "testing_mocks/data_generators.hpp"
#include "types/physics_constants.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class ExtrinsicOptimizationFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        // NOTE(Jack): Requires higher frequency data to actually return a result which makes sense! We do not set these
        // as the default time for the test fixture because then the tests take too long. It is important this gets
        // updated before we call the data generation functions below.
        timing_ = TimingParameters{11, 10, 20};

        camera_id_ = context_.assets.cameras.front().id;
        imu_id_ = context_.assets.imu->id;  // Unprotected optional access!
        camera_info_id_ = InsertCameraInfo(camera_id_);
        targets_id_ = InsertExtractedTargets(camera_id_);
        intrinsics_id_ = InsertIntrinsic(camera_id_);
        poses_id_ = InsertPoses(camera_id_, targets_id_);
        std::tie(imu_data_id_, spline_id_) = InsertImuSetup(camera_id_);
        extrinsic_init_id_ = InsertImuCamExtrinsic(imu_id_, camera_id_);
    }

    AssetId camera_id_;
    AssetId imu_id_;
    StepId camera_info_id_;
    StepId targets_id_;
    StepId intrinsics_id_;
    StepId poses_id_;
    StepId imu_data_id_;
    StepId spline_id_;
    StepId extrinsic_init_id_;
};

TEST_F(ExtrinsicOptimizationFixture, TestExtrinsicOptimizationStepRunner) {
    steps::ExtrinsicOptimization const step{camera_id_,      imu_id_,        targets_id_, imu_data_id_,       1,
                                            camera_info_id_, intrinsics_id_, spline_id_,  extrinsic_init_id_, db_};
    StepId const step_id{RunStep<steps::ExtrinsicOptimization>(context_.workflow_id, step, db_)};

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, imu_id_, camera_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);  // Heuristic!

    auto const result2{database::GravitySelect(db_.get(), step_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_NEAR(result2->norm(), kGravity, 1e-3);  // Heuristic!
}

TEST_F(ExtrinsicOptimizationFixture, TestExtrinsicOptimizationStep) {
    steps::ExtrinsicOptimization const step{camera_id_,      imu_id_,        targets_id_, imu_data_id_,       1,
                                            camera_info_id_, intrinsics_id_, spline_id_,  extrinsic_init_id_, db_};

    EXPECT_EQ(step.Type(), StepType::ExtrinsicOptimization);
    std::vector const gt_assets{camera_id_, imu_id_};
    EXPECT_EQ(step.Assets(), gt_assets);
    EXPECT_EQ(step.CacheKey().value, "eab0d38464e832d533dce71257fb939d1691ff44144e082e05de35a4f14eb551");

    // Build the actual database step id and execute the step.
    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::ExtrinsicInit, "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, imu_id_, camera_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);

    auto const result2{database::GravitySelect(db_.get(), step_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_NEAR(result2->norm(), kGravity, 1e-3);
}
