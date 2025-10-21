#include "testing_mocks/mvg_generator.hpp"

#include "constants.hpp"
#include "eigen_utilities/grid.hpp"
#include "sphere_trajectory.hpp"
#include "spline/se3_spline.hpp"

namespace reprojection::testing_mocks {

MvgGenerator::MvgGenerator(bool const flat, Eigen::Matrix3d const& K)
    : K_{K}, se3_spline_{constants::t0_ns, constants::delta_t_ns} {
    std::vector<Eigen::Isometry3d> const poses{SphereTrajectory(CameraTrajectory{{0, 0, 0}, 1.0, {0, 0, 5}})};
    for (auto const& pose : poses) {
        se3_spline_.AddKnot(pose);
    }

    int const rows{5};
    int const cols{rows};
    int const num_points{rows * cols};
    Eigen::ArrayX2i const grid{eigen_utilities::GenerateGridIndices(rows, cols, false)};
    points_ = Eigen::MatrixX3d::Zero(num_points, 3);
    points_.leftCols(2) = grid.cast<double>();
    // Center around zero with unit range [-0.5, 0.5]
    points_.col(0) = points_.col(0) / (rows - 1);
    points_.col(0) = points_.col(0).array() - 0.5;
    points_.col(1) = points_.col(1) / (cols - 1);
    points_.col(1) = points_.col(1).array() - 0.5;

    if (not flat) {
        // Add z-axis values in range [-0.5, 0.5]
        points_.col(2) = Eigen::VectorXd::Random(num_points) / 2;
    }
}

// Input is fractional time of trajectory from [0,1)
MvgFrame MvgGenerator::Generate(double const t) const {
    assert(0 <= t and t < 1);

    // NOTE(Jack): Look how the "spline_time" is calculated here using constants::num_poses. You see that the fractional
    // trajectory time t is converted back into a "metric" spline time in nanoseconds. However, because the spline
    // requires points at each end for interpolation (one at the start three at the end), but our sphere trajectory does
    // not include those points, we need to shorten the valid times/poses here by spline::constants::k - 1. That makes
    // sure that the fractional spline time calculated from t always yields a valid pose, but that we will be missing
    // one delta_t at the start and three delta_ts at the ends of the sphere trajectory. Why do we go through all this
    // hassle here?
    //
    // (1) Because this test fixture should just work, and I do not want people to go through the trouble of checking if
    // this Generate() function returns a valid value or not.
    // (2) Because we can handle the problem here entirely in this one line of code. If we solved it another way for
    // example extending the sphere trajectory to create points at the ends to facilitate the interpolation, or
    // returning an optional value from Generate() to signal failure, we would need to change many places in the code
    // base and therefore the hack that this really is would be harder to track.
    uint64_t const spline_time{
        constants::t0_ns +
        static_cast<uint64_t>((constants::num_poses - (spline::constants::k - 1)) * constants::delta_t_ns * t)};

    auto const pose_t{se3_spline_.Evaluate(spline_time)};
    assert(pose_t.has_value());  // See note above

    Eigen::MatrixX2d const pixels{Project(points_, K_, pose_t.value())};

    // WARN(Jack): This assumes that all points are always visible! With careful engineering for the default value
    // of K this will be true, but that cannot be guaranteed for all K!!!
    return {pose_t.value(), pixels, points_};
}

Eigen::Matrix3d MvgGenerator::GetK() const { return K_; }

Eigen::MatrixX2d MvgGenerator::Project(Eigen::MatrixX3d const& points_w, Eigen::Matrix3d const& K,
                                       Eigen::Isometry3d const& tf_co_w) {
    // TODO(Jack): Do we need to transform isometries into matrices before we use them? Otherwise it might not
    // match our expectations about matrix dimensions after the fact.
    // TODO(Jack): Should we use the pinhole projection from the nonlinear refinement optimization here?
    Eigen::MatrixX4d const points_homog_co{(tf_co_w * points_w.rowwise().homogeneous().transpose()).transpose()};
    Eigen::MatrixX2d const pixels{(K * points_homog_co.leftCols(3).transpose()).transpose().rowwise().hnormalized()};

    return pixels;
}

}  // namespace reprojection::testing_mocks