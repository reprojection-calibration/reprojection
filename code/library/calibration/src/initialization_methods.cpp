#include "calibration/initialization_methods.hpp"

#include <ranges>

#include "projection_functions/initialize_camera.hpp"

#include "camera_imu_initialization.hpp"
#include "intrinsic_initialization.hpp"
#include "linear_pose_initialization.hpp"

namespace reprojection::calibration {

// TODO(Jack): This method is can eat up time because it runs pnp on every single intrinsic estimate. For example a
//  single 6 by 7 target can give 42 gammas for each image, lets say you have 1000 images then that means we need to
//  run pnp 42000 times... Clearly that is overkill! We should investigate a strategy here to stop searching once we
//  have found a good enough result, or anything to reduce the number of evaluations we need to perform.
std::optional<ArrayXd> InitializeIntrinsics(CameraModel const camera_model, double const height, double const width,
                                            CameraMeasurements const& targets) {
    auto const [runner, initialization]{SelectInitializationStrategy(camera_model, height, width)};

    double min_cost{std::numeric_limits<double>::max()};
    ArrayXd intrinsics;
    for (auto const& target : targets | std::views::values) {
        std::vector<double> const gammas{runner(target)};

        for (auto const gamma_i : gammas) {
            ArrayXd const intrinsics_i{initialization(gamma_i, height, width)};
            ImageBounds const image_bounds{0, width, 0, height};
            auto const camera{projection_functions::InitializeCamera(camera_model, intrinsics_i, image_bounds)};

            auto const result{EstimatePoseViaPinholePnP(camera, target.bundle, image_bounds)};
            if (result.has_value()) {
                auto const [_, final_cost]{*result};
                if (final_cost < min_cost) {
                    min_cost = final_cost;
                    intrinsics = intrinsics_i;
                }
            }
        }
    }

    return intrinsics;
}

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

std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> EstimateCameraImuRotationAndGravity(
    spline::CubicBSplineC3 const& camera_orientation, ImuMeasurements const& imu_data) {
    auto const imu_angular_velocity{ExtractAngularVelocity(imu_data)};
    auto const [R_imu_co, diagnostics]{EstimateCameraImuRotation(camera_orientation, imu_angular_velocity)};

    auto const imu_linear_acceleration{ExtractLinearAcceleration(imu_data)};
    Vector3d const gravity_w{EstimateGravity(camera_orientation, imu_linear_acceleration, R_imu_co)};

    return {{R_imu_co, diagnostics}, gravity_w};
}

}  // namespace reprojection::calibration
