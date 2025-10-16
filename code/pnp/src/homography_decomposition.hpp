#include <Eigen/Dense>

namespace reprojection::pnp {

Eigen::Isometry3d FullPipeline(Eigen::MatrixX2d const& pixels, Eigen::MatrixX3d const& points);

// NOTE(Jack): We should be able to use the same normalization here we used for the DLT
// pixels, 2d_points
std::tuple<Eigen::Vector3d, Eigen::Matrix3d> FindHomography(Eigen::MatrixX2d const& points_src,
                                                            Eigen::MatrixX2d const& points_dst);

// TOOD PROPER NAME
std::tuple<Eigen::MatrixX2d, Eigen::Isometry3d> NormalizePointsForHomographySolving(Eigen::MatrixX3d const& points);

// TODO(Jack): Not tested, is it even possible to test in a reasonable way?
std::tuple<Eigen::Vector3d, Eigen::Matrix3d> WhatDoWeNameThis(Eigen::MatrixX3d const& points);

// TODO THIS FUNCTION ACTUALLY BELONGS IN THE CORE PNP LOGIC TO DECIDE IF WE USE DLT OR HOMOGRAPHY DECOMPOSITION
bool IsPlane(Eigen::MatrixX3d const& points);

}  // namespace reprojection::pnp