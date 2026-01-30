#include "testing_mocks/mvg_generator.hpp"

#include "constants.hpp"
#include "eigen_utilities/grid.hpp"
#include "geometry/lie.hpp"
#include "noise_generation.hpp"
#include "projection_functions/camera_model.hpp"
#include "projection_functions/intialize_camera.hpp"
#include "sphere_trajectory.hpp"
#include "spline/se3_spline.hpp"

namespace reprojection::testing_mocks {

// TODO(Jack): Have a common initialization framework for the camera parameters here and with CameraCalibrationData and
// the camera construction itself. This should just be one struct...
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
    MatrixX3d const points{MvgGenerator::BuildTargetPoints(flat)};

    CameraCalibrationData data{{"/mvg_test_data", camera_model, bounds}, intrinsics};
    for (int i{0}; i < num_frames; ++i) {
        double const elapsed_trajectory{static_cast<double>(i) / num_frames};
        assert(0 <= elapsed_trajectory and elapsed_trajectory < 1);

        uint64_t const spline_time{constants::t0_ns +
                                   static_cast<uint64_t>((num_control_points - spline::constants::degree) *
                                                         constants::delta_t_ns * elapsed_trajectory)};

        auto const pose_t{se3_spline.Evaluate(spline_time)};
        assert(pose_t.has_value());  // TODO REFACTOR TO THROW HERE IF THIS IS NOT VALID!

        auto const [pixels, mask]{MvgGenerator::Project(points, camera, geometry::Exp(pose_t.value()))};
        ArrayXi const valid_indices{eigen_utilities::MaskToRowId(mask)};

        uint64_t const timestamp_ns{constants::t0_ns + constants::delta_t_ns * i};
        data.frames[timestamp_ns].extracted_target.bundle =
            Bundle{pixels(valid_indices, Eigen::all), points(valid_indices, Eigen::all)};
        data.frames[timestamp_ns].initial_pose = pose_t.value();
    }

    return data;
}

/**
 * \brief Static helper method that projects points in the world frame. Do NOT use outside the testing mocks context!
 *
 * This method is intended only for use as part of the testing mocks test data generation class
 * (reprojection::testing_mocks::MvgGenerator) and should NOT be used by other consuming code. It was left as a public
 * method only so that it could be tested.
 */
std::tuple<MatrixX2d, ArrayXb> MvgGenerator::Project(MatrixX3d const& points_w,
                                                     std::unique_ptr<projection_functions::Camera> const& camera,
                                                     Isometry3d const& tf_co_w) {
    MatrixX4d const points_homog_co{(tf_co_w * points_w.rowwise().homogeneous().transpose()).transpose()};

    return camera->Project(points_homog_co.leftCols(3));
}

MatrixX3d MvgGenerator::BuildTargetPoints(bool const flat) {
    int const size{5};  // Square target - rows == cols
    int const num_points{size * size};

    ArrayX2i const grid{eigen_utilities::GenerateGridIndices(size, size, false)};
    MatrixX3d points{MatrixX3d::Zero(num_points, 3)};

    // Center around zero with unit range [-0.5, 0.5]
    points.leftCols(2) = grid.cast<double>();
    points.leftCols(2).array() /= (size - 1);
    points.leftCols(2).array() -= 0.5;

    if (not flat) {
        points.col(2) = VectorXd::Random(num_points) / 2;  // Add random z-axis values in range [-0.5, 0.5]
    }

    return points;
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