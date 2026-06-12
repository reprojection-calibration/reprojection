#include "testing_mocks/mvg_data_generator.hpp"

#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "projection_functions/camera_model.hpp"
#include "projection_functions/initialize_camera.hpp"
#include "testing_mocks/new_sphere_trajectory.hpp"


#include "mvg_helpers.hpp"
#include "noise_generation.hpp"

namespace reprojection::testing_mocks {

// TODO(Jack): Use refactored types to pass to this function instead of all individual args!
// TODO(Jack): Have a common initialization framework for the camera parameters here and with CameraCalibrationData and
//  the camera construction itself. This should just be one struct...?
// TODO(Jack): 99% of uses of this function do not care at all about the timespan_ns, does it make sense to give this a
//  default parameter value and only pass it if we need it?
//
// NOTE(Jack): There are two things to consider in this method; (1) the underlying spline that builds the sphere
// trajectory and (2) the actual poses we sample from it. To prevent aliasing/sampling artifacts in the sampled poses
// we dynamically set num_control_points in the underlying spline to a multiple of num_samples. If we did not do this
// and num_control_points was less than num_samples we would see "saw tooth" like artifacts in the sampled frame poses.
// What value the multiple needs to be is not certain, but for a first try we heuristically found that 5*num_samples was
// sufficient to build a spline that we can sample num_samples from without artifacts.
std::pair<CameraMeasurements, Frames> GenerateMvgData(CameraInfo const& sensor, CameraState const& intrinsics,
                                                      int const num_samples, uint64_t const timespan_ns,
                                                      bool const flat) {
    (void)num_samples;
    (void)timespan_ns;
    auto const [frames, _]{Trajectory2(60, 30, {1.5, 1.5, 1.5}, {0, 0, 0}, 1.0)};

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