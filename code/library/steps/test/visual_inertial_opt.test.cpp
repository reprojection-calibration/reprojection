#include "steps/visual_inertial_opt.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"
#include "types/physics_constants.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class VisualInertialOptFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        StepTestFixture::SetUp();

        // NOTE(Jack): Requires higher frequency data to actually return a result which makes sense! We do not set these
        // as the default time for the test fixture because then the tests take too long. It is important this gets
        // updated before we call the data generation functions below.
        timing_ = TimingParameters{11, 10, 20};

        cam_id_ = context_.assets.cameras.front().id;
        StepId const targets_id{InsertExtractedTargets(cam_id_)};
        imu_id_ = context_.assets.imu->id;  // Unprotected optional access!
        std::tie(imu_data_id_, spline_id_) = InsertImuSetup(cam_id_);

        // NOTE(Jack): We need to insert both the imu-cam and cam-cam extrinsic. In a real application one of these
        // usually comes from the stereo init and another from the inertial init but here we just put them both under
        // the same step id.
        extrinsic_init_id_ = InsertImuCamExtrinsic(imu_id_, cam_id_);
        database::StepCacheKeyUpdate(db_.get(), extrinsic_init_id_, "");
        InsertExtrinsic(cam_id_, cam_id_);

        stage_ids_ = CamStageIds{cam_id_, InsertCameraInfo(cam_id_), targets_id, InsertPoses(cam_id_, targets_id),
                                 InsertIntrinsic(cam_id_)};
    }

    AssetId cam_id_;
    AssetId imu_id_;
    StepId imu_data_id_;
    StepId spline_id_;
    StepId extrinsic_init_id_;
    CamStageIds stage_ids_;
};

TEST_F(VisualInertialOptFixture, TestVisualInertialOptStepRunner) {
    steps::VisualInertialOpt const step{
        imu_id_, imu_data_id_, cam_id_, {stage_ids_}, spline_id_, extrinsic_init_id_, extrinsic_init_id_, 1, db_};
    StepId const step_id{RunStep<steps::VisualInertialOpt>(context_.workflow_id, step, db_)};

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, imu_id_, cam_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);

    auto const result2{database::GravitySelect(db_.get(), step_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_NEAR(result2->norm(), kGravity, 1e-3);
}

TEST_F(VisualInertialOptFixture, TestVisualInertialOptStep) {
    steps::VisualInertialOpt const step{
        imu_id_, imu_data_id_, cam_id_, {stage_ids_}, spline_id_, extrinsic_init_id_, extrinsic_init_id_, 1, db_};

    EXPECT_EQ(step.Type(), StepType::VisualInertialOpt);
    std::vector const gt_assets{imu_id_, cam_id_};
    EXPECT_EQ(step.Assets(), gt_assets);
    EXPECT_EQ(step.CacheKey().value, "5c7bb34547a0aac16e1408fe2c8fe3c515ec51fd5d6d777c4ef995205bd9f8bd");

    // Build the actual database step id and execute the step.
    StepId const step_id{database::GetOrCreateStep(db_.get(), steps::VisualInertialOpt::Type(), "").first};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::ExtrinsicSelect(db_.get(), step_id, imu_id_, cam_id_)};
    ASSERT_TRUE(result.has_value());
    EXPECT_LT(result->se3_a_b.sum(), 0.001);

    auto const result2{database::GravitySelect(db_.get(), step_id)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_NEAR(result2->norm(), kGravity, 1e-3);
}
