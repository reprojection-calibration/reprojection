#include "sphere_trajectory.hpp"

#include "geometry/lie.hpp"
#include "spline/se3_spline.hpp"

#include "constants.hpp"

namespace reprojection::testing_mocks {

// TODO DO NOT USE GLOBAL!
Matrix3d const canonical_camera_axes{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};

// TODO(Jack): Make the distinction between what is a world frame and what is a camera pose.
std::vector<Vector6d> SphereTrajectory(int const num_poses, CameraTrajectory const& config) {
    MatrixX3d const pose_origins{SpherePoints(num_poses, config.sphere_radius, config.sphere_origin)};

    std::vector<Vector6d> tfs;
    for (int i{0}; i < pose_origins.rows(); ++i) {
        Vector3d const position_i{pose_origins.row(i)};
        Matrix3d const direction_i{TrackPoint(config.world_origin, position_i)};

        Isometry3d tf_i;
        tf_i.linear() = direction_i * canonical_camera_axes.inverse();
        tf_i.translation() = position_i;

        tfs.push_back(geometry::Log(tf_i));
    }

    return tfs;
}

// See the code here for a possible implementation https://plotly.com/python/3d-camera-controls/
// NOTE(Jack): I went down a long rabbit hole of trying to understand this method better. Basically the problem that
// explains why we do not get any excitation in the roll (rz) is because we are hard coding this function to always look
// parallel to the worlds z-direction. This does not reflect my original intention of having the camera (z-axis forward)
// looking at one point during the entire trajectory. The bottom line is that if we want excitation of the roll
// dimension using this kind of method we can either heuristically add some roll, or instead fix the direction of
// gravity. If the direction of gravity is fixed then we can use some cross products to get an orthogonal rotation
// matrix that points in the direction we want.
Matrix3d TrackPoint(Vector3d const& origin, Vector3d const& camera_position) {
    Vector3d const forward{origin - camera_position};
    if (forward.norm() < 1e-8) {
        // TODO(Jack): Is this properly handled when the origin and camera_position are the same?
        return Matrix3d::Identity();
    }
    Vector3d const forward_direction{forward.normalized()};

    Vector3d gravity_direction{0, 0, -1};
    if (std::abs(forward_direction.dot(gravity_direction)) > 0.999) {
        gravity_direction = {0, 1, 0};
    }

    // TODO(Jack): Handle the case where forward and gravity are in the same direction!
    Vector3d const xxx{forward_direction.cross(gravity_direction)};
    Vector3d const xxx_direction{xxx.normalized()};
    Vector3d const yyy{forward_direction.cross(xxx_direction)};
    Vector3d const yyy_direction{yyy.normalized()};

    // TODO NAMING
    Matrix3d R;
    R.col(0) = forward_direction;
    R.col(1) = xxx_direction;
    R.col(2) = yyy_direction;

    return R;
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