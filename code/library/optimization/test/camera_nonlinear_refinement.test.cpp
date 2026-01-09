#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "optimization/nonlinear_refinement.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(OptimizationCameraNonlinearRefinement, TestCameraNonlinearRefinementBatch) {
    Array4d const intrinsics{600, 600, 360, 240};
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)))};

    int const num_frames{20};
    std::vector<Frame> const mvg_frames{generator.GenerateBatch(num_frames)};

    CameraCalibrationData data{{"", CameraModel::Pinhole}, intrinsics, {}, {}};

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
        Array6d const se3_gt_pose_i{geometry::Log(gt_pose_i)};

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

    CameraCalibrationData data{{"", CameraModel::Pinhole}, intrinsics, {}, {}};

    uint64_t pseudo_time{0};
    for (auto const& mvg_frame_i : mvg_frames) {
        data.frames[pseudo_time].extracted_target.bundle = mvg_frame_i.bundle;
        data.frames[pseudo_time].initial_pose = geometry::Log(mvg_frame_i.pose);

        pseudo_time += 1;
    }

    optimization::CameraNonlinearRefinement(OptimizationDataView(data));

    for (auto const& frame_i : data.frames) {
        Isometry3d const gt_pose_i{gt_poses[frame_i.first]};

        // WARN(Jack): Clearly I do not understand the axis-angle representation... And here something frustrating
        // happened that I will explain. This test using noisy poses had been working for months, no problems to report.
        // Comparing the Vector6d ses poses directly worked perfectly and the optimization returned the ground truth
        // value. Then suddenly, when we transitioned to the "view" concept, this test started to fail. And what would
        // happen is that the optimized answer would have he right translation but the se3 axis-angle rotation would be
        // flipped (similar to what we observed when plotting the poses in Dash). It was not flipped for all poses, but
        // only sometimes and seemingly randomly. As I still do not have a solution for this, we actually changed this
        // test to instead compare the 4x4 SE3 transformation  matrices. Now it passes again, essentially the same as
        // before, but now working in the matrix space. Why all of a sudden the optimized poses start flipping, I cannot
        // explain.
        EXPECT_TRUE(geometry::Exp(frame_i.second.optimized_pose).isApprox(gt_pose_i, 1e-6))
            << "Nonlinear refinement result:\n"
            << geometry::Exp(frame_i.second.optimized_pose).matrix() << "\nGround truth:\n"
            << gt_pose_i.matrix();
    }

    EXPECT_TRUE(data.optimized_intrinsics.isApprox(intrinsics, 1e-6))
        << "Optimization result:\n"
        << data.optimized_intrinsics << "\noptimization input:\n"
        << intrinsics;
}
