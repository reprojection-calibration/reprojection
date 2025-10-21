#pragma once

#include <Eigen/Dense>

namespace reprojection::testing_mocks {

struct CameraTrajectory {
    Eigen::Vector3d world_origin;
    double sphere_radius;
    Eigen::Vector3d sphere_origin;
};

// Generates an "oriented" trajectory on a sphere. In this case oriented means that the +z-axis is pointed at/looking at
// the world_origin as specified in the CameraTrajectory config. We do this to simulate a camera traveling around in a
// 3D trajectory always facing a single point where the calibration board is.
std::vector<Eigen::Isometry3d> SphereTrajectory(CameraTrajectory const& config);

// Calculates the rotation vector which makes a camera frame (z-axis forward) look at a point. This is used to help
// generate trajectories where the camera is looking at one constant point. This can be used to simulate how a person
// moves a camera while keeping it pointed at the target during the calibration process.
Eigen::Vector3d TrackPoint(Eigen::Vector3d const& origin, Eigen::Vector3d const& camera_position);

// Look at constants.hpp and the implementation here to understand the trajectory that is generated!
Eigen::MatrixX3d SpherePoints(double const radius, Eigen::Vector3d const origin);

Eigen::Vector3d Cartesian(double const theta, double const phi);

}  // namespace reprojection::testing_mocks