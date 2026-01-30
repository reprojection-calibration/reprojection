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
    spline::Se3Spline se3_spline{constants::t0_ns, constants::delta_t_ns};
    for (auto const& pose : SphereTrajectory(constants::num_camera_poses, constants::trajectory)) {
        se3_spline.AddControlPoint(pose);
    }

    auto const camera{projection_functions::InitializeCamera(camera_model, intrinsics, bounds)};
    MatrixX3d const points{MvgGenerator::BuildTargetPoints(flat)};

    CameraCalibrationData data{{"/mvg_test_data", camera_model, bounds}, intrinsics};
    for (int i{0}; i < num_frames; ++i) {
        double const elapsed_trajectory{static_cast<double>(i) / num_frames};
        assert(0 <= elapsed_trajectory and elapsed_trajectory < 1);

        uint64_t const spline_time{constants::t0_ns +
                                   static_cast<uint64_t>((constants::num_camera_poses - (spline::constants::degree)) *
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

MvgGenerator::MvgGenerator(CameraModel const camera_model, ArrayXd const& intrinsics, ImageBounds const& bounds,
                           bool const flat)
    : camera_model_{camera_model},
      intrinsics_{intrinsics},
      bounds_{bounds},
      camera_{projection_functions::InitializeCamera(camera_model_, intrinsics_, bounds_)},
      se3_spline_{constants::t0_ns, constants::delta_t_ns},
      points_{BuildTargetPoints(flat)} {
    std::vector<Isometry3d> const poses{SphereTrajectory(constants::num_camera_poses, constants::trajectory)};

    for (auto const& pose : poses) {
        se3_spline_.AddControlPoint(pose);
    }
}

CameraCalibrationData MvgGenerator::GenerateBatch(int const num_frames) const {
    CameraCalibrationData data{{"mvg_test_data", camera_model_, bounds_}, intrinsics_};
    for (int i{0}; i < num_frames; ++i) {
        auto const [bundle, pose]{this->Generate(static_cast<double>(i) / num_frames)};

        uint64_t const timestamp_ns{constants::t0_ns + constants::delta_t_ns * i};
        data.frames[timestamp_ns].extracted_target.bundle = bundle;
        data.frames[timestamp_ns].initial_pose = geometry::Log(pose);
    }

    return data;
}  // LCOV_EXCL_LINE

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

// Input is fractional time of trajectory from [0,1)
std::tuple<Bundle, Isometry3d> MvgGenerator::Generate(double const t) const {
    assert(0 <= t and t < 1);

    // NOTE(Jack): Look how the "spline_time" is calculated here using constants::num_camera_poses. You see that the
    // fractional trajectory time t is converted back into a "metric" spline time in nanoseconds. However, because the
    // spline requires points at each end for interpolation (one at the start three at the end), but our sphere
    // trajectory does not include those points, we need to shorten the valid times/poses here by spline::constants::k
    // - 1. That makes sure that the fractional spline time calculated from t always yields a valid pose, but that we
    // will be missing one delta_t at the start and three delta_ts at the ends of the sphere trajectory. Why do we go
    // through all this hassle here?
    //
    // (1) Because this test fixture should just work, and I do not want people to go through the trouble of checking if
    // this Generate() function returns a valid value or not.
    // (2) Because we can handle the problem here entirely in this one line of code. If we solved it another way for
    // example extending the sphere trajectory to create points at the ends to facilitate the interpolation, or
    // returning an optional value from Generate() to signal failure, we would need to change many places in the code
    // base and therefore the hack that this really is would be harder to track.
    uint64_t const spline_time{
        constants::t0_ns +
        static_cast<uint64_t>((constants::num_camera_poses - (spline::constants::degree)) * constants::delta_t_ns * t)};

    auto const pose_t{se3_spline_.Evaluate(spline_time)};
    assert(pose_t.has_value());  // See note above

    auto const [pixels, mask]{Project(points_, camera_, geometry::Exp(pose_t.value()))};
    ArrayXi const valid_indices{eigen_utilities::MaskToRowId(mask)};

    // TODO(Jack): With the introduction of the projection mask we are going to have this (indices, Eigen::all) logic
    //  show up in a lot of places! Does it make sense to add a method for it? Maybe masking an entire extracted target
    //  or something like that, but lets see which use cases we run across often before we decide what to do.
    return {{pixels(valid_indices, Eigen::all), points_(valid_indices, Eigen::all)}, geometry::Exp(pose_t.value())};
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