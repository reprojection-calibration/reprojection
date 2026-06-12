#include "testing_mocks/data_generators.hpp"

#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "projection_functions/camera_model.hpp"
#include "projection_functions/initialize_camera.hpp"
#include "testing_mocks/new_sphere_trajectory.hpp"

#include "constants.hpp"
#include "mvg_helpers.hpp"
#include "noise_generation.hpp"

namespace reprojection::testing_mocks {

ImuMeasurements const GenerateImuMeasurements(double const duration_s, double const sample_rate_hz) {
    auto const [_, imu_data]{Trajectory2(duration_s, sample_rate_hz, constants::trajectory.trajectory_center,
                                         constants::trajectory.world_origin, constants::trajectory.sphere_radius)};

    return imu_data;
}

std::pair<CameraMeasurements, Frames> GenerateMvgData(CameraInfo const& sensor, CameraState const& intrinsics,
                                                      double const duration_s, double const sample_rate_hz,
                                                      bool const flat) {
    auto const [frames, _]{Trajectory2(duration_s, sample_rate_hz, constants::trajectory.trajectory_center,
                                       constants::trajectory.world_origin, constants::trajectory.sphere_radius)};

    auto const camera{
        projection_functions::InitializeCamera(sensor.camera_model, intrinsics.intrinsics, sensor.bounds)};
    auto const [points, indices]{MvgHelpers::BuildTargetPoints(flat)};

    CameraMeasurements targets;
    Frames poses;
    for (auto const& [time_ns_i, frame] : frames) {
        Isometry3d const tf_w_b{geometry::Exp(frame.pose)};
        Isometry3d const tf_b_w{tf_w_b.inverse()};

        // The canonical_camera_R here is the classic "z-forward, x-right, y-down" optical camera frame.
        static Matrix3d const R_co_b{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
        Isometry3d const tf_co_w{R_co_b * tf_b_w};

        auto const [pixels, mask]{MvgHelpers::Project(points, camera, tf_co_w)};

        ArrayXi const valid_row_ids{eigen_utilities::MaskToRowId(mask)};
        ExtractedTarget const target_i{Bundle{pixels(valid_row_ids, Eigen::all), points(valid_row_ids, Eigen::all)},
                                       indices(valid_row_ids, Eigen::all)};

        targets.insert({time_ns_i, target_i});
        poses[time_ns_i].pose = geometry::Log(tf_co_w);
    }

    return {targets, poses};
}

Isometry3d AddGaussianNoise(double const sigma_translation, double const sigma_rotation, Isometry3d pose) {
    pose.translation() += GaussianNoise(0, sigma_translation, 3, 1);

    // TODO(Jack): Confirm this is the right way to add noise to a rotation! Adding the gaussian noise element in the
    // tangent space directly and then converting back to a rotation matrix.
    Vector3d const rotation_noise{GaussianNoise(0, sigma_rotation, 3, 1)};
    Vector3d const rotation_se3{geometry::Log<double>(pose.rotation())};
    Vector3d const perturbed_se3{rotation_noise.array() + rotation_se3.array()};

    Matrix3d const perturbed_R{geometry::Exp(perturbed_se3)};
    pose.linear() = perturbed_R;

    return pose;
}

}  // namespace reprojection::testing_mocks