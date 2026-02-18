#include "optimization/camera_nonlinear_refinement.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

#include "projection_cost_function.hpp"

using namespace reprojection;

TEST(OptimizationCameraNonlinearRefinement, TestCameraNonlinearRefinementBatch) {
    CameraCalibrationData const gt_data{testing_mocks::GenerateMvgData(
        50, 1e9, CameraModel::Pinhole, testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds, false)};
    CameraCalibrationData data{gt_data};

    optimization::CeresState const state{optimization::CameraNonlinearRefinement(OptimizationDataView(data))};
    EXPECT_EQ(state.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);

    for (auto const& [timestamp_ns, frame_i] : data.frames) {
        // WARN(Jack): Unprotected optional access! Do we need a better strategy here? The mvg test data should
        // definitely have filled out this value!
        Array6d const gt_aa_co_w{gt_data.frames.at(timestamp_ns).initial_pose.value()};

        ASSERT_TRUE(frame_i.optimized_pose.has_value());
        Array6d const aa_co_w{frame_i.optimized_pose.value()};
        EXPECT_TRUE(aa_co_w.isApprox(gt_aa_co_w, 1e-6)) << "Result:\n"
                                                        << aa_co_w.transpose() << "\nexpected result:\n"
                                                        << gt_aa_co_w.transpose();

        // We are testing with perfect input data so the mean reprojection error before and after optimization is near
        // zero.
        ASSERT_TRUE(frame_i.initial_reprojection_error.has_value());
        EXPECT_NEAR(frame_i.initial_reprojection_error.value().mean(), 0.0, 1e-6);
        ASSERT_TRUE(frame_i.optimized_reprojection_error.has_value());
        EXPECT_NEAR(frame_i.optimized_reprojection_error.value().mean(), 0.0, 1e-6);
    }

    EXPECT_TRUE(data.optimized_intrinsics.isApprox(gt_data.initial_intrinsics, 1e-6))
        << "Result:\n"
        << data.optimized_intrinsics << "\nexpected result:\n"
        << gt_data.initial_intrinsics;
}

// Given a noisy initial pose but perfect bundle (i.e. no noise in the pixels or points), we then get perfect poses
// and intrinsic back.
TEST(OptimizationCameraNonlinearRefinement, TestNoisyCameraNonlinearRefinement) {
    CameraCalibrationData const gt_data{testing_mocks::GenerateMvgData(
        20, 1e9, CameraModel::Pinhole, testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds, false)};
    CameraCalibrationData data{gt_data};

    // Add gaussian noise to the initial poses
    for (auto& [_, frame_i] : data.frames) {
        Isometry3d const SE3_i{geometry::Exp(frame_i.initial_pose.value())};
        frame_i.initial_pose = geometry::Log(testing_mocks::AddGaussianNoise(0.5, 0.5, SE3_i));
    }

    optimization::CeresState const state{optimization::CameraNonlinearRefinement(OptimizationDataView(data))};
    EXPECT_EQ(state.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);

    for (auto const& [timestamp_ns, frame_i] : data.frames) {
        // WARN(Jack): Clearly I do not understand the axis-angle representation... And here something frustrating
        // happened that I will explain. This test using noisy poses had been working for months, no problems to report.
        // Comparing the Vector6d se3 poses directly worked perfectly and the optimization returned the ground truth
        // value. Then suddenly, when we transitioned to the "view" concept, this test started to fail. And what would
        // happen is that the optimized answer would have he right translation but the se3 axis-angle rotation would be
        // flipped (similar to what we observed when plotting the poses in Dash). It was not flipped for all poses, but
        // only sometimes and seemingly randomly. As I still do not have a solution for this, we actually changed this
        // test to instead compare the 4x4 SE3 transformation  matrices. Now it passes again, essentially the same as
        // before, but now working in the matrix space. Why all of a sudden the optimized poses start flipping, I cannot
        // explain.
        // WARN(Jack): Unprotected optional access! Do we need a better strategy here? The mvg test data should
        // definitely have filled out this value!
        Isometry3d const gt_tf_co_w{geometry::Exp(gt_data.frames.at(timestamp_ns).initial_pose.value())};

        ASSERT_TRUE(frame_i.optimized_pose.has_value());
        Isometry3d const tf_co_w{geometry::Exp(frame_i.optimized_pose.value())};

        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w, 1e-6)) << "Result:\n"
                                                        << tf_co_w.matrix() << "\nexpected result:\n"
                                                        << gt_tf_co_w.matrix();

        ASSERT_TRUE(frame_i.optimized_reprojection_error.has_value());
        EXPECT_LT(frame_i.optimized_reprojection_error.value().mean(), 1e-6);
    }

    EXPECT_TRUE(data.optimized_intrinsics.isApprox(gt_data.initial_intrinsics, 1e-6))
        << "Result:\n"
        << data.optimized_intrinsics << "\nexpected result:\n"
        << gt_data.initial_intrinsics;
}

TEST(OptimizationCameraNonlinearRefinement, TestEvaluateReprojectionResiduals) {
    // NOTE(Jack): The real ground truth value for both the valid pixels here is actually the center of the image! But
    // because we want to see that the reprojection error is actually the correct value we make the "ground truth"
    // pixels here actually have some error.
    MatrixX2d const gt_pixels{{-1, -1},  //
                              {350, 230},
                              {-1, -1},
                              {-1, -1},
                              {365, 245}};
    MatrixX3d const gt_points{{0, 0, -600},  //
                              {0, 0, 600},
                              {0, 0, -600},
                              {0, 0, -600},
                              {0, 0, 600}};
    // If the pixel evaluation fails then the cost function will automatically fill out the residual value with 256,
    // this is arbitrary and heuristic. See the note in the projection cost function implementation.
    ArrayX2d const gt_residuals{{256, 256},  //
                                {-10, -10},
                                {256, 256},
                                {256, 256},
                                {5, 5}};

    std::vector<std::unique_ptr<ceres::CostFunction>> cost_functions;
    for (Eigen::Index j{0}; j < gt_pixels.rows(); ++j) {
        cost_functions.emplace_back(optimization::Create(CameraModel::Pinhole, testing_utilities::image_bounds,
                                                         gt_pixels.row(j), gt_points.row(j)));
    }

    ArrayX2d const residuals{optimization::EvaluateReprojectionResiduals(
        cost_functions, testing_utilities::pinhole_intrinsics, {0, 0, 0, 0, 0, 0})};
    EXPECT_TRUE(residuals.isApprox(gt_residuals));
}