#pragma once

#include <Eigen/Dense>

namespace reprojection::testing_mocks {

// TODO(Jack): Not all/any the methods currently in this file belong in the public interface!

struct CameraTrajectory {
    Eigen::Vector3d world_origin;
    double sphere_radius;
    Eigen::Vector3d sphere_origin;

    // uint64_t t0_ns;
    // uint64_t tend_ns;
};

Eigen::Vector3d Cartesian(double const theta, double const phi);

Eigen::MatrixX3d SpherePoints(double const radius, Eigen::Vector3d const origin);

// COPY AND PASTED FROM ORIGINAL DATA GENERATOR
Eigen::Vector3d TrackPoint(Eigen::Vector3d const& origin, Eigen::Vector3d const& camera_position);

std::vector<Eigen::Isometry3d> SphereTrajectory(CameraTrajectory const& config);

}  // namespace reprojection::testing_mocks