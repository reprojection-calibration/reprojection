#include "sphere_trajectory.hpp"

#include "constants.hpp"
#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/se3_spline.hpp"

namespace reprojection::testing_mocks {

std::vector<Eigen::Isometry3d> SphereTrajectory(CameraTrajectory const& config) {
    Eigen::MatrixX3d const sphere_points{SpherePoints(config.sphere_radius, config.sphere_origin)};

    std::vector<Eigen::Isometry3d> tfs;
    for (int i{0}; i < sphere_points.rows(); ++i) {
        Eigen::Vector3d const point_i{sphere_points.row(i)};
        Eigen::Vector3d const camera_direction{TrackPoint(config.world_origin, point_i)};

        Eigen::Isometry3d tf_i;
        tf_i.linear() = geometry::Exp(camera_direction);
        tf_i.translation() = point_i;

        tfs.push_back(tf_i);
    }

    return tfs;
}

Eigen::Vector3d TrackPoint(Eigen::Vector3d const& origin, Eigen::Vector3d const& camera_position) {
    Eigen::Vector3d const delta{origin - camera_position};
    if (delta.norm() < 1e-8) {
        // The origin and camera_position are the same point
        return Eigen::Vector3d::Zero();
    }

    Eigen::Vector3d const origin_direction{delta.normalized()};
    Eigen::Vector3d const camera_forward_direction{0, 0, 1};  // Camera standard z-axis forward

    double const dot{origin_direction.dot(camera_forward_direction)};
    double const angle{std::acos(dot)};

    Eigen::Vector3d const cross_product{camera_forward_direction.cross(origin_direction)};
    double const cross_norm{cross_product.norm()};

    // Edge case handlign when camera position and origin lie on the same vertical line (i.e. case when x and y
    // coordinates are the same for both).
    if (cross_norm < 1e-8) {
        if (dot > 0.9999999) {
            // Parallel case
            return Eigen::Vector3d::Zero();
        } else {
            // Anti-parallel case
            return angle * Eigen::Vector3d::UnitX();  // Could also use the y-axis
        }
    }

    Eigen::Vector3d const axis{cross_product / cross_norm};
    Eigen::Vector3d const tracking_direction{angle * axis};

    return tracking_direction;
}

Eigen::MatrixX3d SpherePoints(double const radius, Eigen::Vector3d const origin) {
    // TODO(Jack): These should be globals in the context of the test mocking
    int const points{constants::num_poses};
    int const loops{constants::num_loops};

    Eigen::MatrixX3d sphere(points, 3);
    for (int i{0}; i < points; ++i) {
        double const theta{2 * M_PI * loops * i / points};
        double const phi{2 * M_PI * i / points};

        sphere.row(i) = origin + (radius * Cartesian(theta, phi));
    }

    return sphere;
}  // LCOV_EXCL_LINE

Eigen::Vector3d Cartesian(double const theta, double const phi) {
    double const x{std::sin(theta) * std::cos(phi)};
    double const y{std::sin(theta) * std::sin(phi)};
    double const z{std::cos(theta)};

    return {x, y, z};
}

}  // namespace reprojection::testing_mocks