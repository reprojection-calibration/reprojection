#include "sphere_trajectory.hpp"

#include "geometry/lie.hpp"
#include "spline/se3_spline.hpp"

#include "constants.hpp"

namespace reprojection::testing_mocks {

// NOTE(Jack): This method produces tf_w_co poses, but most applications want tf_co_w. I am not 100% sure that we cannot
// just directly return tf_w_co here, but the unit test testing the sphere properties (i.e. zero mean, radius etc.) was
// easier to write when the transform was tf_w_co because then we could subtract the center directly from it. My hope is
// that when we dig into the IMU we will get a better idea of the right choice.
std::vector<Vector6d> SphereTrajectory(int const num_poses, CameraTrajectory const& config) {
    MatrixX3d const pose_origins{SpherePoints(num_poses, config.sphere_radius, config.sphere_origin)};

    // The canonical_camera_R here is the classic "z-forward, x-right, y-down" optical camera frame.
    static Matrix3d const canonical_camera_R{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};

    // NOTE(Jack): These are the values which will be incrementally updated as we traverse the pose_origins by calling
    // the TrackPoint method for each new position.
    Matrix3d R{canonical_camera_R.inverse()};
    Vector3d forward{Vector3d::UnitX()};

    std::vector<Vector6d> poses;
    for (int i{0}; i < pose_origins.rows(); ++i) {
        Vector3d const position_i{pose_origins.row(i)};
        std::tie(R, forward) = TrackPoint(config.world_origin, position_i, R, forward);

        Isometry3d tf_w_co;
        tf_w_co.linear() = R;
        tf_w_co.translation() = position_i;

        poses.push_back(geometry::Log(tf_w_co));
    }

    return poses;
}

std::tuple<Matrix3d, Vector3d> TrackPoint(Vector3d const& origin, Vector3d const& camera_position,
                                          Matrix3d const& R_prev, Vector3d const& forward_prev) {
    Vector3d const forward{origin - camera_position};
    if (forward.norm() < 1e-8) {
        return {R_prev, forward_prev};
    }

    Vector3d const axis{forward_prev.cross(forward)};
    double const sin_angle{axis.norm()};
    if (sin_angle < 1e-8) {
        return {R_prev, forward_prev};
    }

    double const cos_angle{forward_prev.dot(forward)};
    double const angle{std::atan2(sin_angle, cos_angle)};

    Eigen::AngleAxisd const aa(angle, axis.normalized());
    Matrix3d const R_delta{aa.toRotationMatrix()};

    Matrix3d const R_new{R_delta * R_prev};
    Vector3d const forward_new{forward.normalized()};

    return {R_new, forward_new};
}

MatrixX3d SpherePoints(int const num_points, double const radius, Vector3d const origin) {
    MatrixX3d sphere(num_points, 3);
    for (int i{0}; i < num_points; ++i) {
        double const theta{2 * M_PI * constants::num_loops * i / num_points};
        double const phi{2 * M_PI * i / num_points};

        sphere.row(i) = origin + (radius * Cartesian(theta, phi));
    }

    return sphere;
}  // LCOV_EXCL_LINE

Vector3d Cartesian(double const theta, double const phi) {
    double const x{std::sin(theta) * std::cos(phi)};
    double const y{std::sin(theta) * std::sin(phi)};
    double const z{std::cos(theta)};

    return {x, y, z};
}

}  // namespace reprojection::testing_mocks