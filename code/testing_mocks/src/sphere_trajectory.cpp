#include "sphere_trajectory.hpp"

#include "constants.hpp"
#include "geometry/lie.hpp"
#include "spline/constants.hpp"
#include "spline/se3_spline.hpp"

namespace reprojection::testing_mocks {

Eigen::Vector3d Cartesian(double const theta, double const phi) {
    double const x{std::sin(theta) * std::cos(phi)};
    double const y{std::sin(theta) * std::sin(phi)};
    double const z{std::cos(theta)};

    return {x, y, z};
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
}

// COPY AND PASTED FROM ORIGINAL DATA GENERATOR
// WARN(Jack): In testing this function I found that when the x and y coordinates of the origin and camera_position
// are the same, i.e. the points  are aligned along the z-plane, the algorithm returns nans. There is probably a
// simple test we can use to check this condition to avoid the nans, but for now we will just take this risk and
// hope that we never have the same x and y coordinates for both camera and origin. This current implementation also
// does not explicitly handle the case where the two points are the same, I assume that takes some error handling to
// prevent nans as well.
Eigen::Vector3d TrackPoint(Eigen::Vector3d const& origin, Eigen::Vector3d const& camera_position) {
    Eigen::Vector3d const origin_direction{(origin - camera_position).normalized()};
    Eigen::Vector3d const camera_forward_direction{0, 0, 1};

    double const angle{std::acos(origin_direction.transpose() * camera_forward_direction)};

    Eigen::Vector3d const cross_product{camera_forward_direction.cross(origin_direction)};
    Eigen::Vector3d const axis{cross_product / cross_product.norm()};

    Eigen::Vector3d const tracking_direction{angle * axis};

    return tracking_direction;
}

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

}  // namespace reprojection::testing_mocks