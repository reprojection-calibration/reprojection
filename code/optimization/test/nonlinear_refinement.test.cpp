#include "optimization/nonlinear_refinement.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

// TODO(Jack): Test the nonlinear refinement with noisy data to make sure the optimization executes more than one step!

TEST(OptimizationNonlinearRefinement, TestNonlinearRefinementBatch) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};

    int const num_frames{20};
    std::vector<Frame> const frames{generator.GenerateBatch(num_frames)};

    auto const [poses_opt, K]{optimization::NonlinearRefinement(frames, CameraModel::Pinhole, intrinsics)};

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
}

TEST(OptimizationNonlinearRefinement, TestNoisyNonlinearRefinement) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};

    // Given a perfect bundle (i.e. pixel and point noise sigmas=0) but noisy initial pose, we then get perfect poses
    // and intrinsic back.
    auto const [gt_frames, noisy_frames]{generator.GenerateBatchWithNoise(20, {0, 0, 0.5, 0.5})};
    auto const [poses_opt, K]{optimization::NonlinearRefinement(noisy_frames, CameraModel::Pinhole, intrinsics)};
    for (size_t i{0}; i < std::size(poses_opt); ++i) {
        auto const pose_opt_i{poses_opt[i]};
        auto const pose_gt_i{gt_frames[i].pose};

        EXPECT_TRUE(pose_opt_i.isApprox(pose_gt_i, 1e-6))
            << "Optimization result:\n"
            << geometry::Log(pose_opt_i).transpose() << "\nGround truth:\n"
            << geometry::Log(pose_gt_i).transpose();
    }

    EXPECT_TRUE(K.isApprox(intrinsics, 1e-6)) << "Optimization result:\n"
                                              << K << "\noptimization input:\n"
                                              << intrinsics;
}
