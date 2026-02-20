#include "calibration/linear_pose_initialization.hpp"

#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "pnp/pnp.hpp"
#include "projection_functions/camera_model.hpp"
#include "projection_functions/intialize_camera.hpp"

namespace reprojection::calibration {

// Doxygen notes: only work because we have same camera center for the pinhole and ds/other camera model used. The goal
// of the function is to unproject the pixels to 3d rays using a roughly initialized camera, then project these back to
// pixels using an ideal unit pinhole camera, which essentially undistorts them. Now that we have data that comes from
// an equivalent pinhole camera we can apply dlt/pnp and get an initial pose.
OptimizationState LinearPoseInitialization(CameraInfo const& sensor, CameraMeasurements const& targets,
                                           CameraState const& intrinsics) {
    OptimizationState linear_solution;
    for (auto const& [timestamp_ns, target_i] : targets) {
        // Unproject to rays (pseudo 3D - no depth information) using the camera model provided by the user.
        auto const camera{
            projection_functions::InitializeCamera(sensor.camera_model, intrinsics.intrinsics, sensor.bounds)};
        MatrixX3d const rays{camera->Unproject(target_i.bundle.pixels)};

        // Project the rays using a unit ideal pinhole camera to get undistorted/linearized pixels
        auto const pinhole_camera{projection_functions::PinholeCamera({1, 1, 0, 0}, {-1, 1, -1, 1})};
        auto const [pixels, mask]{pinhole_camera.Project(rays)};

        // TOOD(Jack): Now we have this similar masking logic here and in the mvg generator. Should we use a function
        // instead?
        ArrayXi const valid_indices{eigen_utilities::MaskToRowId(mask)};
        Bundle const linearized_bundle{pixels(valid_indices, Eigen::all),
                                       target_i.bundle.points(valid_indices, Eigen::all)};

        // This is actually a logically critical point, because a frame only gets added to the optimization state if pnp
        // was successful. If it failed then the frame is not optimized and we need to make sure all our downstream
        // logic handles this fact, that just because a target is there does not mean that an initial pose/frame needs
        // to be.
        auto const result{pnp::Pnp(linearized_bundle, sensor.bounds)};
        if (std::holds_alternative<Isometry3d>(result)) {
            linear_solution.frames[timestamp_ns].pose = geometry::Log(std::get<Isometry3d>(result));
        }
    }

    linear_solution.camera_state = intrinsics;

    return linear_solution;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::calibration
