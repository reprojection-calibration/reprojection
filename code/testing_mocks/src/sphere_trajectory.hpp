#pragma once

#include <Eigen/Dense>

namespace reprojection::testing_mocks {

struct CameraTrajectory {
    Eigen::Vector3d world_origin;
    double sphere_radius;
    Eigen::Vector3d sphere_origin;
};

std::vector<Eigen::Isometry3d> SphereTrajectory(CameraTrajectory const& config);

Eigen::Vector3d TrackPoint(Eigen::Vector3d const& origin, Eigen::Vector3d const& camera_position);

Eigen::MatrixX3d SpherePoints(double const radius, Eigen::Vector3d const origin);

Eigen::Vector3d Cartesian(double const theta, double const phi);

}  // namespace reprojection::testing_mocks