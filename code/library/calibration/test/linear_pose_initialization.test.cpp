#include "calibration/linear_pose_initialization.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/sensor_types.hpp"

using namespace reprojection;

TEST(CalibrationLinearPoseInitialization, TestXXX) {
    Array6d const double_sphere_intrinsics{600, 600, 360, 240, 0.1, 0.2};

    // Generate fisheye test data
    // TODO(Jack): Find a way to use the same camera for both calls!
    testing_mocks::MvgGenerator const generator{
        testing_mocks::MvgGenerator(std::unique_ptr<projection_functions::Camera>(
            new projection_functions::DoubleSphereCamera(double_sphere_intrinsics)))};

    int const num_frames{20};
    std::vector<Frame> const frames{generator.GenerateBatch(num_frames)};

    std::set<ExtractedTargetStamped> targets;
    uint64_t pseudo_time{0};
    for (auto const& frame : frames) {
        // WARN(Jack): Indices is left blank even though technically the indices size should always match the bundle!!!
        targets.insert({{pseudo_time, "/cam/retro/123"}, {frame.bundle, ArrayX2i{}}});

        pseudo_time += 1;
    }

    // naming!!!
    // MAKE EXPLICIT THAT WE ARE ACTUALLY USEING THE 3d pnp here I think!!!
    auto const poses{calibration::LinearPoseInitialization(
        targets, std::unique_ptr<projection_functions::Camera>(
                     new projection_functions::DoubleSphereCamera(double_sphere_intrinsics)))};

    // WARN(Jack): Using num frames here assumes that the linear pose initialization was successful for all frames. This
    // will not always be true and is only valid here in the controlled testing scenario.
    // MAKE EXPLCIIT THAT WE ARE HACKING THE PSEUDO TIME HERE FOR THE PURPOSE OF THE TEST!!!!
    for (auto const& stamped_pose_i : poses) {
        Isometry3d const gt_pose_i{frames[stamped_pose_i.header.timestamp_ns].pose};
        Vector6d const se3_gt_pose_i{geometry::Log(gt_pose_i.inverse())};

        EXPECT_TRUE(stamped_pose_i.pose.isApprox(se3_gt_pose_i, 1e-6))
            << "Linear pose initialization result:\n"
            << stamped_pose_i.pose.transpose() << "\nGround truth:\n"
            << se3_gt_pose_i.transpose();
    }
}
