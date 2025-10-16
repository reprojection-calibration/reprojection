#include <Eigen/Dense>

namespace reprojection::pnp {

// TODO(Jack): Not tested, is it even possible to test in a reasonable way?
std::tuple<Eigen::Vector3d, Eigen::Matrix3d> WhatDoWeNameThis(Eigen::MatrixX3d const& points);

// TODO THIS FUNCTION ACTUALLY BELONGS IN THE CORE PNP LOGIC TO DECIDE IF WE USE DLT OR HOMOGRAPHY DECOMPOSITION
bool IsPlane(Eigen::MatrixX3d const& points);

}  // namespace reprojection::pnp