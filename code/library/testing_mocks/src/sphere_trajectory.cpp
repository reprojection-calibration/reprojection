#include "sphere_trajectory.hpp"

#include "constants.hpp"
#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/se3_spline.hpp"

namespace reprojection::testing_mocks {

std::vector<Isometry3d> SphereTrajectory(int const num_poses, CameraTrajectory const& config) {
    MatrixX3d const pose_origins{SpherePoints(num_poses, config.sphere_radius, config.sphere_origin)};

    std::vector<Isometry3d> tfs;
    for (int i{0}; i < pose_origins.rows(); ++i) {
        Vector3d const position_i{pose_origins.row(i)};
        Vector3d const camera_direction{TrackPoint(config.world_origin, position_i)};

        Isometry3d tf_i;
        tf_i.linear() = geometry::Exp(camera_direction);
        tf_i.translation() = position_i;

        tfs.push_back(tf_i);
    }

    return tfs;
}

Vector3d TrackPoint(Vector3d const& origin, Vector3d const& camera_position) {
    Vector3d const delta{origin - camera_position};
    if (delta.norm() < 1e-8) {
        // The origin and camera_position are the same point
        return Vector3d::Zero();
    }

    Vector3d const origin_direction{delta.normalized()};
    Vector3d const camera_forward_direction{0, 0, 1};  // Camera standard z-axis forward

    double const dot{origin_direction.dot(camera_forward_direction)};
    double const angle{std::acos(dot)};

    Vector3d const cross_product{camera_forward_direction.cross(origin_direction)};
    double const cross_norm{cross_product.norm()};

    // Edge case handlign when camera position and origin lie on the same vertical line (i.e. case when x and y
    // coordinates are the same for both).
    if (cross_norm < 1e-8) {
        if (dot > 0.9999999) {
            // Parallel case
            return Vector3d::Zero();
        } else {
            // Anti-parallel case
            return angle * Vector3d::UnitX();  // Could also use the y-axis
        }
    }

    Vector3d const axis{cross_product / cross_norm};
    Vector3d const tracking_direction{angle * axis};

    return tracking_direction;
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