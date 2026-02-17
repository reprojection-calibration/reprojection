#include "sphere_trajectory.hpp"

#include <iostream>

#include "geometry/lie.hpp"
#include "spline/se3_spline.hpp"

#include "constants.hpp"

namespace reprojection::testing_mocks {

// TODO DO NOT USE GLOBAL!
Matrix3d const canonical_camera_axes{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};

// TODO(Jack): Make the distinction between what is a world frame and what is a camera pose.
std::vector<Vector6d> SphereTrajectory(int const num_poses, CameraTrajectory const& config) {
    MatrixX3d const pose_origins{SpherePoints(num_poses, config.sphere_radius, config.sphere_origin)};

    // TODO(Jack): We randomly chose these initial values. Do we require a principled way to do this? Are there any
    //  limitations that this selection of initial values has? Will it also work for all sphere trajectory
    //  paramaterizations?
    Matrix3d R_prev{Matrix3d::Identity()};
    Vector3d forward_prev{Vector3d::Identity()};

    std::vector<Vector6d> tfs;
    for (int i{0}; i < pose_origins.rows(); ++i) {
        Vector3d const position_i{pose_origins.row(i)};
        std::tie(R_prev, forward_prev) = TrackPoint(config.world_origin, position_i, R_prev, forward_prev);

        Isometry3d tf_i;
        tf_i.linear() = R_prev * canonical_camera_axes.inverse();
        tf_i.translation() = position_i;

        tfs.push_back(geometry::Log(tf_i));
    }

    return tfs;
}

// Calculates the shortest arc rotation between the two forward direction vectors - prevents jumps and discontinuities.
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