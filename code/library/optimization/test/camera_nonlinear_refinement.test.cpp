#include <gtest/gtest.h>

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
    std::vector<Frame> const mvg_frames{generator.GenerateBatch(num_frames)};

    CameraSensorData data{{"", CameraModel::Pinhole}, intrinsics, {}, {}};

    // TODO(Jack): Refactor mvg generator to use new calibration types??? Or does that not make any sense? At least
    // provide an adaptor that converts frames into the data field of the dict.
    uint64_t pseudo_time{0};
    for (auto const& mvg_frame_i : mvg_frames) {
        // Unlike real extracted targets this target has 3D points (not 2D z=0 target points) and does not have indices.
        data.frames[pseudo_time].extracted_target.bundle = mvg_frame_i.bundle;
        data.frames[pseudo_time].initial_pose = geometry::Log(mvg_frame_i.pose);

        pseudo_time += 1;
    }

    optimization::CameraNonlinearRefinement(OptimizationDataView(data));

    for (auto const& frame_i : data.frames) {
        // WARN(Jack): we are abusing the psuedo timestamp from the frame map to index into a vector. Hack!
        Isometry3d const gt_pose_i{mvg_frames[frame_i.first].pose};
        Vector6d const se3_gt_pose_i{geometry::Log(gt_pose_i)};

        EXPECT_TRUE(frame_i.second.optimized_pose.isApprox(se3_gt_pose_i, 1e-6))
            << "Nonlinear refinement result:\n"
            << frame_i.second.optimized_pose.transpose() << "\nGround truth:\n"
            << se3_gt_pose_i.transpose();
    }

    EXPECT_TRUE(data.optimized_intrinsics.isApprox(intrinsics, 1e-6))
        << "Optimization result:\n"
        << data.optimized_intrinsics << "\noptimization input:\n"
        << intrinsics;
}

// Given a perfect bundle (i.e. no noise in the pixels or points) but noisy initial pose, we then get perfect poses
// and intrinsic back.
TEST(OptimizationCameraNonlinearRefinement, TestNoisyCameraNonlinearRefinement) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};

    auto mvg_frames{generator.GenerateBatch(20)};
    std::vector<Isometry3d> gt_poses;  // Store the poses from the frames because we are about to add noise to the frame
    for (auto& mvg_frame_i : mvg_frames) {
        gt_poses.push_back(mvg_frame_i.pose);
        mvg_frame_i.pose = testing_mocks::AddGaussianNoise(0.5, 0.5, mvg_frame_i.pose);
    }

    CameraSensorData data{{"", CameraModel::Pinhole}, intrinsics, {}, {}};

    uint64_t pseudo_time{0};
    for (auto const& mvg_frame_i : mvg_frames) {
        data.frames[pseudo_time].extracted_target.bundle = mvg_frame_i.bundle;
        data.frames[pseudo_time].initial_pose = geometry::Log(mvg_frame_i.pose);

        pseudo_time += 1;
    }

    optimization::CameraNonlinearRefinement(OptimizationDataView(data));

    for (auto const& frame_i : data.frames) {
        std::cout << frame_i.first << std::endl;
        Isometry3d const gt_pose_i{gt_poses[frame_i.first]};
        Vector6d const se3_gt_pose_i{geometry::Log(gt_pose_i)};

        EXPECT_TRUE(frame_i.second.optimized_pose.isApprox(se3_gt_pose_i, 1e-6))
            << "Nonlinear refinement result:\n"
            << frame_i.second.optimized_pose.transpose() << "\nGround truth:\n"
            << se3_gt_pose_i.transpose();
    }

    EXPECT_TRUE(data.optimized_intrinsics.isApprox(intrinsics, 1e-6))
        << "Optimization result:\n"
        << data.optimized_intrinsics << "\noptimization input:\n"
        << intrinsics;
}
