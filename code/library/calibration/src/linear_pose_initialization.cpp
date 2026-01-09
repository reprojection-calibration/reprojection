#include "calibration/linear_pose_initialization.hpp"

#include "geometry/lie.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"
#include "types/sensor_types.hpp"

namespace reprojection::calibration {

// TODO MOVE TO FOLDER
// TODO ADD SANITY CHECKS?
// TODO MAKE SURE THIS IS NOT ALREADY IMPLEMENTED SOMEHWERE
// TODO(Jack): Add all other camera models and check the above listed TODO points.
std::unique_ptr<projection_functions::Camera> InitializeCamera(CameraModel model, ArrayXd const& intrinsics) {
    if (model == CameraModel::DoubleSphere) {
        return std::unique_ptr<projection_functions::Camera>(new projection_functions::DoubleSphereCamera(intrinsics));
    } else {
        throw std::runtime_error("invalid camera model");  // LVOV_EXCL_LINE
    }
}

// Doxygen notes: only work because we have same camera center for the pinhole and ds/other camera model used. The goal
// of the function is to unproject the pixels to 3d rays using a roughly initialized camera, then project these back to
// pixels using an ideal unit pinhole camera, which essentially undistorts them. Now that we have data that comes from
// an equivalent pinhole camera we can apply dlt/pnp and get an initial pose.
void LinearPoseInitialization(InitializationDataView data_view) {
    auto const camera{InitializeCamera(data_view.camera_model(), data_view.initial_intrinsics())};

    for (InitializationFrameView frame_i : data_view) {
        MatrixX3d const rays{camera->Unproject(frame_i.extracted_target().bundle.pixels)};

        // Project using a unit ideal pinhole camera to get pseudo undistorted pixels
        auto const pinhole_camera{projection_functions::PinholeCamera({1, 1, 0, 0})};
        // ERROR(Jack): We are not accounting for the fact of valid field of view!
        MatrixX2d const pixels{pinhole_camera.Project(rays)};
        Bundle const linearized_bundle{pixels, frame_i.extracted_target().bundle.points};

        auto const result{pnp::Pnp(linearized_bundle)};
        // TODO(Jack): What is a principled way to handle errors and communicate that to the user? Right now the
        //  returned set will just be missing values. This can also be valid, and I think as we are using sets the user
        //  should already be aware of he fact that they must use the set key for correspondence and not simply the
        //  position in the container.
        if (not std::holds_alternative<Isometry3d>(result)) {
            continue;  // LCOV_EXCL_LINE
        }

        // ERROR(Jack): What is the proper place to do this inverse? Do we need this inverse at all really? When we have
        // some consistency in our coordinate systems we can decide this.
        Vector6d const se3_i{geometry::Log(std::get<Isometry3d>(result).inverse())};  // INVERSE!!!

        // cppcheck-suppress unreadVariable
        frame_i.initial_pose() = se3_i;
    }
}

}  // namespace reprojection::calibration
