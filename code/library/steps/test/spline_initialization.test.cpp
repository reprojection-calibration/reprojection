#include "steps/spline_initialization.hpp"

#include <gtest/gtest.h>

#include "steps/step_runner.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

#include "test_fixture.hpp"

using namespace reprojection;

class SplineInitFixture : public StepTestFixture {
   protected:
    void SetUp() override {
        CameraInfo const camera_info{CameraModel::DoubleSphere, testing_utilities::image_bounds};
        camera_info_id_ = InsertCameraInfo(camera_info);

        auto const [targets, poses]{
            testing_mocks::GenerateMvgData(camera_info, {testing_utilities::double_sphere_intrinsics}, 11, 1)};
        targets_id_ = InsertExtractedTargets(targets);

        database::CameraPosesInsert(db_.get(), pose_init_id_, targets_id_, camera_id_, poses);
    }

    StepId camera_info_id_;
    StepId pose_init_id_{database::GetOrCreateStep(db_.get(), StepType::PoseInit, "").first};
    StepId targets_id_;
    // cppcheck-suppress unusedStructMember
    StepId intrinsics_id_{
        InsertIntrinsics(CameraModel::DoubleSphere, CameraState{testing_utilities::double_sphere_intrinsics})};
};

TEST_F(SplineInitFixture, TestSplineInitStepRunner) {
    steps::SplineInitialization const step{camera_id_,      pose_init_id_,  targets_id_,
                                           camera_info_id_, intrinsics_id_, db_};
    StepId const step_id{RunStep<steps::SplineInitialization>(workflow_id_, step, db_)};

    auto const result{database::ControlPointsSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 3978);  // Heuristic

    auto const result2{database::SplineInfoSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_EQ(result2->t0_ns_, 2200000000);     // Heuristic
    EXPECT_EQ(result2->delta_t_ns_, 10000000);  // Heuristic
}

TEST_F(SplineInitFixture, TestSplineInitStep) {
    steps::SplineInitialization const step{camera_id_,      pose_init_id_,  targets_id_,
                                           camera_info_id_, intrinsics_id_, db_};
    EXPECT_EQ(step.Type(), StepType::SplineInit);
    EXPECT_EQ(step.CacheKey().value, "46d20a41437bc2c70b8497e5d0cebef0fcfb8bed7854b6e4ac1f1664ef006d02");

    auto const [step_id, _]{database::GetOrCreateStep(db_.get(), StepType::SplineInit, "")};
    EXPECT_NO_THROW(step.Execute(step_id, db_));

    auto const result{database::ControlPointsSelect(db_.get(), step_id, camera_id_)};
    EXPECT_EQ(std::size(result), 3978);

    auto const result2{database::SplineInfoSelect(db_.get(), step_id, camera_id_)};
    ASSERT_TRUE(result2.has_value());
    EXPECT_EQ(result2->t0_ns_, 2200000000);
    EXPECT_EQ(result2->delta_t_ns_, 10000000);
}
