#include "calibration/linear_pose_initialization.hpp"

#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"

namespace reprojection::calibration {

// TODO MOVE TO FOLDER
// TODO ADD SANITY CHECKS?
// TODO MAKE SURE THIS IS NOT ALREADY IMPLEMENTED SOMEHWERE
// TODO(Jack): Add all other camera models and check the above listed TODO points.
std::unique_ptr<projection_functions::Camera> InitializeCamera(CameraModel const model, ImageBounds const& bounds,
                                                               ArrayXd const& intrinsics) {
    if (model == CameraModel::DoubleSphere) {
        return std::unique_ptr<projection_functions::Camera>(
            new projection_functions::DoubleSphereCamera(intrinsics, bounds));
    } else {
        throw std::runtime_error("invalid camera model");  // LCOV_EXCL_LINE
    }
}

// Doxygen notes: only work because we have same camera center for the pinhole and ds/other camera model used. The goal
// of the function is to unproject the pixels to 3d rays using a roughly initialized camera, then project these back to
// pixels using an ideal unit pinhole camera, which essentially undistorts them. Now that we have data that comes from
// an equivalent pinhole camera we can apply dlt/pnp and get an initial pose.
void LinearPoseInitialization(InitializationDataView data_view) {
    auto const camera{
        InitializeCamera(data_view.camera_model(), data_view.image_bounds(), data_view.initial_intrinsics())};

    for (InitializationFrameView frame_i : data_view) {
        // Project using a unit ideal pinhole camera to get pseudo undistorted pixels
        MatrixX3d const rays{camera->Unproject(frame_i.extracted_target().bundle.pixels)};
        auto const pinhole_camera{projection_functions::PinholeCamera({1, 1, 0, 0}, {-1, 1, -1, 1})};
        auto const [pixels, mask]{pinhole_camera.Project(rays)};

        // TOOD(Jack): Now we have this similar masking logic here and in the mvg generator. Should we use a function
        // instead?
        ArrayXi const valid_indices{eigen_utilities::MaskToRowId(mask)};
        Bundle const linearized_bundle{pixels(valid_indices, Eigen::all),
                                       frame_i.extracted_target().bundle.points(valid_indices, Eigen::all)};

        auto const result{pnp::Pnp(linearized_bundle)};
        if (std::holds_alternative<Isometry3d>(result)) {
            // ERROR(Jack): What is the proper place to do this inverse? Do we need this inverse at all really? When we
            // have some consistency in our coordinate systems we can decide this.
            Vector6d const se3_i{geometry::Log(std::get<Isometry3d>(result).inverse())};  // INVERSE!!!
            frame_i.initial_pose() = se3_i;  // cppcheck-suppress unreadVariable
        } else {
            frame_i.initial_pose() = std::nullopt;  // cppcheck-suppress unreadVariable // LCOV_EXCL_LINE
        }
    }
}

}  // namespace reprojection::calibration
