#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace reprojection {

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using VectorXd = Eigen::VectorXd;

using Matrix3d = Eigen::Matrix3d;

using MatrixX2d = Eigen::MatrixX2d;
using MatrixX3d = Eigen::MatrixX3d;

using ArrayXd = Eigen::ArrayXd;
using ArrayXi = Eigen::ArrayXi;
using ArrayX2i = Eigen::ArrayX2i;
using Array2d = Eigen::Array2d;
using Array3d = Eigen::Array3d;
using Array4d = Eigen::Array4d;
using Array5d = Eigen::Array<double, 5, 1>;
using Array6d = Eigen::Array<double, 6, 1>;
using Array8d = Eigen::Array<double, 8, 1>;

using Isometry3d = Eigen::Isometry3d;

};  // namespace reprojection
