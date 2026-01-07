#include "calibration/linear_pose_initialization.hpp"

#include "geometry/lie.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"
#include "types/sensor_types.hpp"

namespace reprojection::calibration {

std::set<PoseStamped> LinearPoseInitialization(std::set<ExtractedTargetStamped> const& targets,
                                               std::unique_ptr<projection_functions::Camera const> const& camera) {
    std::set<PoseStamped> poses;
    for (auto const& [header, extracted_target] : targets) {
        MatrixX3d const rays{camera->Unproject(extracted_target.bundle.pixels)};

        // Project using a unit ideal pinhole camera to get pseudo undistorted pixels
        auto const pinhole_camera{projection_functions::PinholeCamera({1, 1, 0, 0})};
        // ERROR(Jack): We are not accounting for the fact of valid field of view!
        MatrixX2d const pixels{pinhole_camera.Project(rays)};
        Bundle const linearized_bundle{pixels, extracted_target.bundle.points};

        auto const result{pnp::Pnp(linearized_bundle)};
        // TODO(Jack): What is a principled way to handle errors and communicate that to the user? Right now the
        //  returned set will just be missing values. This can also be valid, and I think as we are using sets the user
        //  should already be aware of he fact that they must use the set key for correspondence and not simply the
        //  position in the container.
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
