#include "testing_mocks/mvg_data_generator.hpp"

#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "projection_functions/camera_model.hpp"
#include "projection_functions/intialize_camera.hpp"
#include "spline/se3_spline.hpp"

#include "constants.hpp"
#include "mvg_helpers.hpp"
#include "noise_generation.hpp"
#include "sphere_trajectory.hpp"

namespace reprojection::testing_mocks {

// TODO(Jack): There is a lot of shared logic here and with the IMU data generation. Is the a reason to simplify it?
// TODO(Jack): Have a common initialization framework for the camera parameters here and with CameraCalibrationData and
//  the camera construction itself. This should just be one struct...?
CameraCalibrationData GenerateMvgData(int const num_frames, CameraModel const camera_model, ArrayXd const& intrinsics,
                                      ImageBounds const& bounds, bool const flat) {
    // NOTE(Jack): There are two things to consider in this method; (1) the underlying spline that builds the sphere
    // trajectory and (2) the actual poses we sample from it. To prevent aliasing/sampling artifacts in the sampled
    // poses we dynamically set num_control_points in the underlying spline to a multiple of num_frames. If we did not
    // do this and num_control_points was less than num_frames we would see "saw tooth" like artifacts in the sampled
    // frame poses. What value the multiple needs to be is not certain, but for a first try we heuristically found that
    // 2*num_frames was sufficient to build a spline that we can sample num_frames from without artifacts.
    int const num_control_points{2 * num_frames};
    spline::Se3Spline se3_spline{constants::t0_ns, constants::delta_t_ns};
    for (auto const& pose : SphereTrajectory(num_control_points, constants::trajectory)) {
        se3_spline.AddControlPoint(pose);
    }

    auto const camera{projection_functions::InitializeCamera(camera_model, intrinsics, bounds)};
    MatrixX3d const points{MvgHelpers::BuildTargetPoints(flat)};

    CameraCalibrationData data{{"/mvg_test_data", camera_model, bounds}, intrinsics};
    for (int i{0}; i < num_frames; ++i) {
        double const elapsed_trajectory{static_cast<double>(i) / num_frames};
        uint64_t const spline_time{constants::t0_ns +
                                   static_cast<uint64_t>((num_control_points - spline::constants::degree) *
                                                         constants::delta_t_ns * elapsed_trajectory)};

        auto const pose_t{se3_spline.Evaluate(spline_time)};
        if (not pose_t.has_value()) {
            throw std::runtime_error("GenerateMvgData() failed se3_spline.Evaluate().");  // LCOV_EXCL_LINE
        }

        auto const [pixels, mask]{MvgHelpers::Project(points, camera, geometry::Exp(pose_t.value()))};
        ArrayXi const valid_indices{eigen_utilities::MaskToRowId(mask)};

        data.frames[spline_time].extracted_target.bundle =
            Bundle{pixels(valid_indices, Eigen::all), points(valid_indices, Eigen::all)};
        data.frames[spline_time].initial_pose = pose_t.value();
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