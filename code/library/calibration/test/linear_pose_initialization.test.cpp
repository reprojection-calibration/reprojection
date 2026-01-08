#include "calibration/linear_pose_initialization.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/sensor_types.hpp"

using namespace reprojection;

TEST(CalibrationLinearPoseInitialization, TestLinearPoseInitialization) {
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

    CameraSensorData cam0{{"/cam/retro/123", CameraModel::DoubleSphere}, Array6d{600, 600, 360, 240, 0.1, 0.2}, {}, {}};

    // TODO(Jack): Refactor mvg generator to use new calibration types??? Or does that not make any sense? At least
    // provide an adaptor that converts frames into the data field of the dict.
    uint64_t pseudo_time{0};
    for (auto const& frame_i : frames) {
        // Unlike real extracted targets this target has 3D points (not 2D z=0 target points) and does not have indices.
        cam0.frames[pseudo_time].extracted_target.bundle = frame_i.bundle;
        pseudo_time += 1;
    }

    // NOTE(Jack): Because the mvg points are 3d (i.e. z!=0) the underlying pnp implementation tested here will use
    // Dlt23 and not Dlt22 which is what is normally used during the calibration process. Technically this does not make
    // a difference, but if we remove the Dlt23 at some later date we might need to change something here.
    calibration::LinearPoseInitialization(InitializationDataView(cam0));

    for (auto const& frame_i : cam0.frames) {
        Isometry3d const gt_pose_i{frames[frame_i.first].pose};
        Vector6d const se3_gt_pose_i{geometry::Log(gt_pose_i.inverse())};  // Note the inverse!

        EXPECT_TRUE(frame_i.second.initial_pose.isApprox(se3_gt_pose_i, 1e-6))
            << "Linear pose initialization result:\n"
            << frame_i.second.initial_pose.transpose() << "\nGround truth:\n"
            << se3_gt_pose_i.transpose();
    }
}
