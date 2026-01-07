#include "calibration/linear_pose_initialization.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/sensor_types.hpp"

using namespace reprojection;

TEST(CalibrationLinearPoseInitialization, TestXXX) {
    // TODO(Jack): Find a way to use the same camera for both calls! Instead of using the same intrinsics can we
    //  actually just make one camera and use that in both places? Right now we compromise by building both cameras from
    //  the same intrinsics.
    Array6d const double_sphere_intrinsics{600, 600, 360, 240, 0.1, 0.2};

    // Generate fisheye test data so we can prove that the function correctly handles and undistorts non-pinhole input
    // data.
    testing_mocks::MvgGenerator const generator{
        testing_mocks::MvgGenerator(std::unique_ptr<projection_functions::Camera>(
            new projection_functions::DoubleSphereCamera(double_sphere_intrinsics)))};
    int const num_frames{20};
    std::vector<Frame> const frames{generator.GenerateBatch(num_frames)};

    // Convert the mvg data into pseudo extracted targets. Unlike real extracted targets these targets have 3D points
    // (not 2D z=0 target points) and do not have any indices.
    std::set<ExtractedTargetStamped> targets;
    uint64_t pseudo_time{0};
    for (auto const& frame : frames) {
        // WARN(Jack): Indices is left blank even though technically the indices size should always match the bundle!!!
        // Maybe we will add a constructor check to Target that enforces these sizes match and will need to fix this
        // here.
        targets.insert({{pseudo_time, "/cam/retro/123"}, {frame.bundle, ArrayX2i{}}});

        pseudo_time += 1;
    }

    // NOTE(Jack): Because the mvg points are 3d (i.e. z!=0) the underlying pnp implementation tested here will use
    // Dlt23 and not Dlt22 which is what is normally used during the calibration process. Technically this does not make
    // a difference, but if we remove the Dlt23 at some later date we might need to change something here.
    auto const poses{calibration::LinearPoseInitialization(
        targets, std::unique_ptr<projection_functions::Camera>(
                     new projection_functions::DoubleSphereCamera(double_sphere_intrinsics)))};

    for (auto const& stamped_pose_i : poses) {
        // WARN(Jack): We are assuming but not checking that all frames were successfully initialized. If this is no
        // true then the way we abuse the timestamp to index into frames will cause a segfault. This abuse of the
        // timestamp is not acceptable in any application code, and even here in the testing it will likely cause us
        // pain in the future!
        Isometry3d const gt_pose_i{frames[stamped_pose_i.header.timestamp_ns].pose};
        Vector6d const se3_gt_pose_i{geometry::Log(gt_pose_i.inverse())}; // Note the inverse!

        EXPECT_TRUE(stamped_pose_i.pose.isApprox(se3_gt_pose_i, 1e-6))
            << "Linear pose initialization result:\n"
            << stamped_pose_i.pose.transpose() << "\nGround truth:\n"
            << se3_gt_pose_i.transpose();
    }
}
