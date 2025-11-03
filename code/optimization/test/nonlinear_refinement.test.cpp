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

    std::vector<MatrixX2d> pixels;
    std::vector<MatrixX3d> points;
    // TODO(Jack): Find a canonical way to add noise!
    std::vector<Isometry3d> poses_input;  // Will get added noise
    std::vector<Isometry3d> poses_gt;
    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};
        pixels.push_back(frame_i.pixels);
        points.push_back(frame_i.points);
        poses_input.push_back(frame_i.pose);
        poses_gt.push_back(frame_i.pose);

        poses_input.back().translation() += (0.1 * Eigen::Vector3d::Random(3));
    }

    auto const [poses_opt,
                K]{optimization::NonlinearRefinement(pixels, points, poses_input, CameraModel::Pinhole, intrinsics)};

    for (size_t i{0}; i < std::size(poses_opt); ++i) {
        auto const pose_opt_i{poses_opt[i]};
        auto const pose_gt_i{poses_gt[i]};

        EXPECT_TRUE(pose_opt_i.isApprox(pose_gt_i, 1e-6))
            << "Optimization result:\n"
            << geometry::Log(pose_opt_i).transpose() << "\nGround truth:\n"
            << geometry::Log(pose_gt_i).transpose() << "\nOptimization input:\n"
            << geometry::Log(poses_input[i]).transpose();
    }

    EXPECT_TRUE(K.isApprox(intrinsics, 1e-6)) << "Optimization result:\n"
                                              << K << "\noptimization input:\n"
                                              << intrinsics;
}

TEST(OptimizationNonlinearRefinement, TestNonlinearRefinement) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};
    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};

        auto const [tf, K]{optimization::NonlinearRefinement(frame_i.pixels, frame_i.points, frame_i.pose,
                                                             CameraModel::Pinhole, intrinsics)};

        EXPECT_TRUE(tf.isApprox(frame_i.pose)) << "Optimization result:\n"
                                               << geometry::Log(tf) << "\noptimization input:\n"
                                               << geometry::Log(frame_i.pose);
        EXPECT_TRUE(K.isApprox(intrinsics)) << "Optimization result:\n" << K << "\noptimization input:\n" << intrinsics;
    }
}
