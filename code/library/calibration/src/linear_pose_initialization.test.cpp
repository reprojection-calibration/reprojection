#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"
#include "testing_mocks/mvg_generator.hpp"
#include "types/sensor_types.hpp"

namespace reprojection::calibration {

// TODO(Jack): Is it proper here that we introduce stamped types? Or can we avoid the dependency on stamps for longer?
std::set<PoseStamped> LinearPoseInitialization(std::set<ExtractedTargetStamped> const& targets,
                                               std::unique_ptr<projection_functions::Camera const> const& camera) {
    std::set<PoseStamped> poses;
    for (auto const& [header, extracted_target] : targets) {
        MatrixX3d const rays{camera->Unproject(extracted_target.bundle.pixels)};

        // ERROR(Jack): We are not accounting for the fact of valid field of view!
        // Project using a unit ideal pinhole camera to get pseudo undistorted pixels
        auto const pinhole_camera{projection_functions::PinholeCamera({1, 1, 0, 0})};
        MatrixX2d const pixels{pinhole_camera.Project(rays)};

        Bundle const linearized_target{pixels, extracted_target.bundle.points};

        auto const result{pnp::Pnp(linearized_target)};
        // TODO(Jack): What is a principled way to handle errors and communicate that to the user? Right now the
        // returned set will just be missing values. This can also be valid, and I think as we are using sets the user
        // should already be aware of he fact that they must use the set key for correspondence and not simply the
        // position in the container.
        if (not std::holds_alternative<Isometry3d>(result)) {
            continue;
        }

        // WARN(Jack): What is the proper place to do this inverse? Do we need this inverse at all really? When we have
        // some consistency in our coordinate systems we can decide this.
        Vector6d const se3_i{geometry::Log(std::get<Isometry3d>(result).inverse())};  // INVERSE!!!

        poses.insert({header, se3_i});
    }

    return poses;
}

}  // namespace reprojection::calibration

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
    for (auto const& pose : poses) {
        Isometry3d const gt_pose_i{frames[pose.header.timestamp_ns].pose};
        Vector6d const se3_gt_pose_i{geometry::Log(gt_pose_i.inverse())};

        EXPECT_TRUE(pose.pose.isApprox(se3_gt_pose_i, 1e-6)) << "Linea pose initialization result:\n"
                                                             << pose.pose.transpose() << "\nGround truth:\n"
                                                             << se3_gt_pose_i.transpose();
    }
}
