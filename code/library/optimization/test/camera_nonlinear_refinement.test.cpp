#include "optimization/camera_nonlinear_refinement.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

#include "projection_cost_function.hpp"

using namespace reprojection;

// Test with perfect data - means inputs will be exact same as outputs. Technically this test might miss something
// because the optimization will likely not even execute once because the error is zero. For a real test look at the
// next case where we add some noisy so it actually does some iterations.
TEST(OptimizationCameraNonlinearRefinement, TestCameraNonlinearRefinementBatch) {
    // Generate the data
    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState const gt_intrinsics{testing_utilities::pinhole_intrinsics};
    auto const [targets, gt_frames]{testing_mocks::GenerateMvgData(sensor, gt_intrinsics, 50, 1e9, false)};

    // Solve
    OptimizationState const initial_state{gt_intrinsics, gt_frames};
    auto const [optimized_state, diagnostics]{optimization::CameraNonlinearRefinement(sensor, targets, initial_state)};
    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);

    // Assert
    EXPECT_EQ(std::size(optimized_state.frames), 50);
    for (auto const& [timestamp_ns, frame_i] : optimized_state.frames) {
        Array6d const gt_aa_co_w{gt_frames.at(timestamp_ns).pose};
        Array6d const aa_co_w{frame_i.pose};
        EXPECT_TRUE(aa_co_w.isApprox(gt_aa_co_w, 1e-6)) << "Result:\n"
                                                        << aa_co_w.transpose() << "\nexpected result:\n"
                                                        << gt_aa_co_w.transpose();
    }

    EXPECT_TRUE(optimized_state.camera_state.intrinsics.isApprox(gt_intrinsics.intrinsics, 1e-6))
        << "Result:\n"
        << optimized_state.camera_state.intrinsics << "\nexpected result:\n"
        << gt_intrinsics.intrinsics;
}

// Given a noisy initial pose but perfect bundle (i.e. no noise in the pixels or points), we then get perfect poses
// and intrinsic back.
TEST(OptimizationCameraNonlinearRefinement, TestNoisyCameraNonlinearRefinement) {
    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState const gt_intrinsics{testing_utilities::pinhole_intrinsics};
    auto const [targets, gt_frames]{testing_mocks::GenerateMvgData(sensor, gt_intrinsics, 50, 1e9, false)};

    // Add gaussian noise to the initial poses
    Frames noisy_frames{gt_frames};
    for (auto& [_, frame_i] : noisy_frames) {
        Isometry3d const SE3_i{geometry::Exp(frame_i.pose)};
        frame_i.pose = geometry::Log(testing_mocks::AddGaussianNoise(0.5, 0.5, SE3_i));
    }

    OptimizationState const initial_state{gt_intrinsics, noisy_frames};
    auto const [optimized_state, diagnostics]{optimization::CameraNonlinearRefinement(sensor, targets, initial_state)};
    EXPECT_EQ(diagnostics.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);

    EXPECT_EQ(std::size(optimized_state.frames), 50);
    for (auto const& [timestamp_ns, frame_i] : optimized_state.frames) {
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
        Isometry3d const gt_tf_co_w{geometry::Exp(gt_frames.at(timestamp_ns).pose)};
        Isometry3d const tf_co_w{geometry::Exp(frame_i.pose)};

        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w, 1e-6)) << "Result:\n"
                                                        << tf_co_w.matrix() << "\nexpected result:\n"
                                                        << gt_tf_co_w.matrix();
    }

    EXPECT_TRUE(optimized_state.camera_state.intrinsics.isApprox(gt_intrinsics.intrinsics, 1e-6))
        << "Result:\n"
        << optimized_state.camera_state.intrinsics.transpose() << "\nexpected result:\n"
        << gt_intrinsics.intrinsics.transpose();
}

TEST(OptimizationCameraNonlinearRefinement, TestEvaluateReprojectionResiduals) {
    // NOTE(Jack): The real ground truth value for both the valid pixels here is actually the center of the image (i.e.
    // [360, 240])! But because we want to see that the reprojection error is actually the correct value we make the
    // "ground truth" pixels here have some error.
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

    uint64_t const timestamp_ns{0};  // Used to track the data frame in the maps

    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraMeasurements const targets{{timestamp_ns, {{gt_pixels, gt_points}, {}}}};
    OptimizationState const state{CameraState{testing_utilities::pinhole_intrinsics},
                                  {{timestamp_ns, {Array6d::Zero()}}}};

    ReprojectionErrors const residuals{optimization::ReprojectionResiduals(sensor, targets, state)};
    EXPECT_TRUE(residuals.at(timestamp_ns).isApprox(gt_residuals))
        << "Result:\n"
        << residuals.at(timestamp_ns).transpose() << "\nexpected result:\n"
        << gt_residuals.transpose();
}