#include "optimization/bundle_adjustment.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

using Ba = optimization::BundleAdjustment;

// Test with perfect data - means inputs will be exact same as outputs. Technically this test might miss something
// because the optimization will likely not even execute once because the error is zero. For a real test look at the
// next case where we add some noisy so it actually does some iterations.
TEST(OptimizationBundleAdjustment, TestBundleAdjustmentBatch) {
    // Generate the data
    CameraInfo const camera_info{CameraModel::Pinhole, testing_utilities::image_bounds};
    Intrinsic const gt_intrinsics{testing_utilities::pinhole_intrinsics};
    auto const [targets, gt_frames]{testing_mocks::GenerateMvgData(camera_info, gt_intrinsics, 60, 1, false)};

    // Construct problem and solve
    Ba::Problem const problem{Ba::SingleCamProblem(camera_info, gt_intrinsics, gt_frames, targets)};
    auto const [frames, ceres_state, cameras]{Ba::Solve(problem, 1)};
    EXPECT_EQ(ceres_state.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);

    // Assert
    EXPECT_EQ(std::size(frames), 56);
    for (auto const& [timestamp_ns, frame_i] : frames) {
        Array6d const gt_se3_co_w{gt_frames.at(timestamp_ns).value};
        Array6d const se3_co_w{frame_i.value};

        EXPECT_TRUE(se3_co_w.isApprox(gt_se3_co_w, 1e-6)) << "Result:\n"
                                                          << se3_co_w.transpose() << "\nexpected result:\n"
                                                          << gt_se3_co_w.transpose();
    }

    // TODO(Jack): This is a super hacky way to recover the value! What if we change the asset id used internally one
    // day! We need to make a single camera result or something like that, or centralize this code in a helper function
    // is we end up using single camera ba in a lot of places!
    auto const intrinsics{cameras.at(AssetId{0}).intrinsic.value};
    EXPECT_TRUE(intrinsics.isApprox(gt_intrinsics.value, 1e-6)) << "Result:\n"
                                                                << intrinsics.transpose() << "\nexpected result:\n"
                                                                << gt_intrinsics.value.transpose();
}

// Given a noisy initial pose but perfect bundle (i.e. no noise in the pixels or points), we then get perfect poses
// and intrinsic back.
TEST(OptimizationBundleAdjustment, TestNoisyBundleAdjustment) {
    CameraInfo const camera_info{CameraModel::Pinhole, testing_utilities::image_bounds};
    Intrinsic const gt_intrinsics{testing_utilities::pinhole_intrinsics};
    auto const [targets, gt_frames]{testing_mocks::GenerateMvgData(camera_info, gt_intrinsics, 60, 1, false)};

    // Add gaussian noise to the initial poses
    Frames noisy_frames{gt_frames};
    for (auto& [_, frame_i] : noisy_frames) {
        Isometry3d const SE3_i{geometry::Exp(frame_i.value)};
        frame_i.value = geometry::Log(testing_mocks::AddGaussianNoise(0.1, 0.1, SE3_i));
    }

    Ba::Problem const problem{Ba::SingleCamProblem(camera_info, gt_intrinsics, noisy_frames, targets)};
    auto const [frames, ceres_state, cameras]{Ba::Solve(problem, 1)};

    EXPECT_EQ(ceres_state.solver_summary.termination_type, ceres::TerminationType::CONVERGENCE);

    EXPECT_EQ(std::size(frames), 56);
    for (auto const& [timestamp_ns, frame_i] : frames) {
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
        Isometry3d const gt_tf_co_w{geometry::Exp(gt_frames.at(timestamp_ns).value)};
        Isometry3d const tf_co_w{geometry::Exp(frame_i.value)};

        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w, 1e-6)) << "Result:\n"
                                                        << tf_co_w.matrix() << "\nexpected result:\n"
                                                        << gt_tf_co_w.matrix();
    }

    // TODO(Jack): See comment in test above about the need for a better asset id independent single camera workflow.
    auto const intrinsics{cameras.at(AssetId{0}).intrinsic.value};
    EXPECT_TRUE(intrinsics.isApprox(gt_intrinsics.value, 1e-6)) << "Result:\n"
                                                                << intrinsics.transpose() << "\nexpected result:\n"
                                                                << gt_intrinsics.value.transpose();
}

TEST(OptimizationBundleAdjustment, TestReprojectionError) {
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

    CameraInfo const camera_info{CameraModel::Pinhole, testing_utilities::image_bounds};
    Intrinsic const intrinsic{testing_utilities::pinhole_intrinsics};
    Frames const frames{{timestamp_ns, {Array6d::Zero()}}};
    TargetSamples const targets{{timestamp_ns, {{gt_pixels, gt_points}, {}}}};

    Ba::Problem const problem{Ba::SingleCamProblem(camera_info, intrinsic, frames, targets)};

    auto const residuals{optimization::ReprojectionError(problem)};
    EXPECT_EQ(std::size(residuals), 1);

    // There is only one value so we hardcode index into the 0 spot. Does not scale but works for the test!
    auto const residual{residuals[0]};
    EXPECT_EQ(residual.camera_id, AssetId{0});
    EXPECT_EQ(residual.timestamp_ns, timestamp_ns);
    EXPECT_TRUE(residual.value.isApprox(gt_residuals)) << "Result:\n"
                                                       << residual.value.transpose() << "\nexpected result:\n"
                                                       << gt_residuals.transpose();
}