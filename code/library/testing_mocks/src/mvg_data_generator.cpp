#include "testing_mocks/mvg_data_generator.hpp"

#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "projection_functions/camera_model.hpp"
#include "projection_functions/intialize_camera.hpp"
#include "spline/se3_spline.hpp"

#include "data_generator_helpers.hpp"
#include "mvg_helpers.hpp"
#include "noise_generation.hpp"

namespace reprojection::testing_mocks {

// TODO(Jack): Have a common initialization framework for the camera parameters here and with CameraCalibrationData and
//  the camera construction itself. This should just be one struct...?
// TODO(Jack): 99% of uses of this function do not care at all about the timespan_ns, does it make sense to give this a
//  default parameter value and only pass it if we need it?
//
// NOTE(Jack): There are two things to consider in this method; (1) the underlying spline that builds the sphere
// trajectory and (2) the actual poses we sample from it. To prevent aliasing/sampling artifacts in the sampled poses
// we dynamically set num_control_points in the underlying spline to a multiple of num_samples. If we did not do this
// and num_control_points was less than num_samples we would see "saw tooth" like artifacts in the sampled frame poses.
// What value the multiple needs to be is not certain, but for a first try we heuristically found that 2*num_samples was
// sufficient to build a spline that we can sample num_samples from without artifacts.
CameraCalibrationData GenerateMvgData(int const num_samples, uint64_t const timespan_ns, CameraModel const camera_model,
                                      ArrayXd const& intrinsics, ImageBounds const& bounds, bool const flat) {
    // NOTE(Jack): Instead of building and using a spline here, I guess we could have all just used the poses from
    // SphereTrajectory() directly (?) But we follow this pattern here and in the imu data generator for consistencies'
    // sake. The more places we use the spline code the more robust it makes it!
    spline::Se3Spline const trajectory{TimedSphereTrajectorySpline(2 * num_samples, timespan_ns)};
    std::set<uint64_t> const times{SampleTimes(num_samples, timespan_ns)};

    auto const camera{projection_functions::InitializeCamera(camera_model, intrinsics, bounds)};
    MatrixX3d const points{MvgHelpers::BuildTargetPoints(flat)};

    CameraCalibrationData data{{"/mvg_test_data", camera_model, bounds}, intrinsics};
    for (auto const time_i : times) {
        auto const pose_t{trajectory.Evaluate(time_i)};
        if (not pose_t.has_value()) {
            throw std::runtime_error("GenerateMvgData() failed trajectory.Evaluate().");  // LCOV_EXCL_LINE
        }

        auto const [pixels, mask]{MvgHelpers::Project(points, camera, geometry::Exp(pose_t.value()))};
        ArrayXi const valid_indices{eigen_utilities::MaskToRowId(mask)};

        data.frames[time_i].extracted_target.bundle =
            Bundle{pixels(valid_indices, Eigen::all), points(valid_indices, Eigen::all)};
        data.frames[time_i].initial_pose = pose_t.value();
    }

    return data;
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