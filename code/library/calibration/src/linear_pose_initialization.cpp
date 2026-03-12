#include "calibration/linear_pose_initialization.hpp"

#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"
#include "projection_functions/intialize_camera.hpp"

namespace reprojection::calibration {

using Camera = projection_functions::Camera;
using PinholeCamera = projection_functions::PinholeCamera;

// TODO USE THE UNPROJECTION MASKING HERE! I THOUGHT WE ALREADY USED THIS HERE! It needs to be combined with the
// projection mask.

// Doxygen notes: only work because we have same camera center for the pinhole and ds/other camera model used. The goal
// of the function is to unproject the pixels to 3d rays using a roughly initialized camera, then project these back to
// pixels using an ideal unit pinhole camera, which essentially undistorts them. Now that we have data that comes from
// an equivalent pinhole camera we can apply dlt/pnp and get an initial pose.
// TODO(Jack): This name is misleading because the process is not actually strictly linear!
Frames LinearPoseInitialization(CameraInfo const& sensor, CameraMeasurements const& targets,
                                CameraState const& intrinsics) {
    auto const camera{
        projection_functions::InitializeCamera(sensor.camera_model, intrinsics.intrinsics, sensor.bounds)};

    Frames linear_solution;
    for (auto const& [timestamp_ns, target_i] : targets) {
        auto const result{EstimatePoseViaPinholePnP(camera, target_i.bundle, sensor.bounds)};

        if (result.has_value()) {
            auto const [pose, _]{*result};
            linear_solution[timestamp_ns] = pose;
        }
    }

    return linear_solution;
}  // LCOV_EXCL_LINE

std::optional<std::pair<FrameState, double>> EstimatePoseViaPinholePnP(std::unique_ptr<Camera> const& camera,
                                                                       Bundle const& bundle,
                                                                       ImageBounds const& bounds) {
    // Unproject to rays (pseudo 3D - no depth information) using the camera model provided by the user.
    auto const [rays, mask_unproject]{camera->Unproject(bundle.pixels)};

    // Project the rays using a unit ideal pinhole camera to get undistorted/linearized pixels
    auto const pinhole_camera{PinholeCamera({1, 1, 0, 0}, {-1, 1, -1, 1})};
    auto const [pixels, mask_project]{pinhole_camera.Project(rays)};

    // Combine the masks and make a new bundle from only the valid reprojected points.
    ArrayXb const mask{mask_unproject * mask_project};
    ArrayXi const valid_indices{eigen_utilities::MaskToRowId(mask)};
    Bundle const linearized_bundle{pixels(valid_indices, Eigen::all), bundle.points(valid_indices, Eigen::all)};

    auto const result{pnp::Pnp(linearized_bundle, bounds)};
    if (std::holds_alternative<pnp::PoseWithCost>(result)) {
        auto const [tf_co_w, cost]{std::get<pnp::PoseWithCost>(result)};
        return std::pair<FrameState, double>{geometry::Log(tf_co_w), cost};
    } else {
        return std::nullopt;
    }
}

}  // namespace reprojection::calibration
