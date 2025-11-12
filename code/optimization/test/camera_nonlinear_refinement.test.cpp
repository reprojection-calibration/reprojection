#include <gtest/gtest.h>

#include "../../testing_mocks/src/noise_generation.hpp"
#include "geometry/lie.hpp"
#include "optimization/nonlinear_refinement.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(OptimizationCameraNonlinearRefinement, TestCameraNonlinearRefinementBatch) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};

    int const num_frames{20};
    std::vector<Frame> const frames{generator.GenerateBatch(num_frames)};

    auto const [poses_opt, K,
                final_cost]{optimization::CameraNonlinearRefinement(frames, CameraModel::Pinhole, intrinsics)};

    for (size_t i{0}; i < std::size(poses_opt); ++i) {
        auto const pose_opt_i{poses_opt[i]};
        auto const pose_i{frames[i].pose};

        EXPECT_TRUE(pose_opt_i.isApprox(pose_i, 1e-6)) << "Optimization result:\n"
                                                       << geometry::Log(pose_opt_i).transpose() << "\nGround truth:\n"
                                                       << geometry::Log(pose_i).transpose();
    }

    EXPECT_TRUE(K.isApprox(intrinsics, 1e-6)) << "Optimization result:\n"
                                              << K << "\noptimization input:\n"
                                              << intrinsics;

    EXPECT_NEAR(final_cost, 0.0, 1e-12);
}

TEST(OptimizationCameraNonlinearRefinement, TestNoisyCameraNonlinearRefinement) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};

    auto frames{generator.GenerateBatch(20)};
    std::vector<Isometry3d> gt_poses;  // Store the poses from the frames because we are about to add noise to the frame
    for (auto& frame : frames) {
        gt_poses.push_back(frame.pose);
        frame.pose = testing_mocks::AddGaussianNoise(0.5, 0.5, frame.pose);
    }

    // Given a perfect bundle (i.e. no noise in the pixels or points) but noisy initial pose, we then get perfect poses
    // and intrinsic back.
    auto const [poses_opt, K,
                final_cost]{optimization::CameraNonlinearRefinement(frames, CameraModel::Pinhole, intrinsics)};
    for (size_t i{0}; i < std::size(poses_opt); ++i) {
        auto const pose_opt_i{poses_opt[i]};
        auto const pose_gt_i{gt_poses[i]};

        EXPECT_TRUE(pose_opt_i.isApprox(pose_gt_i, 1e-6))
            << "Optimization result:\n"
            << geometry::Log(pose_opt_i).transpose() << "\nGround truth:\n"
            << geometry::Log(pose_gt_i).transpose();
    }

    EXPECT_TRUE(K.isApprox(intrinsics, 1e-6)) << "Optimization result:\n"
                                              << K << "\noptimization input:\n"
                                              << intrinsics;
    EXPECT_NEAR(final_cost, 0.0, 1e-12);
}
