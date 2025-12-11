#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace reprojection {

template <typename T>
using Vector3 = Eigen::Vector3<T>;

using Vector2i = Eigen::Vector2i;
using Vector2d = Eigen::Vector2d;
using Vector3d = Vector3<double>;
using Vector4d = Eigen::Vector4d;
using Vector6d = Eigen::Vector<double, 6>;
using VectorXd = Eigen::VectorXd;

template <typename T>
using Matrix3 = Eigen::Matrix3<T>;

using Matrix2d = Eigen::Matrix2d;
using Matrix3d = Matrix3<double>;
using Matrix4d = Eigen::Matrix4d;
using MatrixXi = Eigen::MatrixXi;
using MatrixXd = Eigen::MatrixXd;
using MatrixX2d = Eigen::MatrixX2d;
using MatrixX3d = Eigen::MatrixX3d;
using MatrixX4d = Eigen::MatrixX4d;

using ArrayXd = Eigen::ArrayXd;
using ArrayXi = Eigen::ArrayXi;
using ArrayX2i = Eigen::ArrayX2i;
using Array2i = Eigen::Array2i;
using Array4i = Eigen::Array4i;
using Array2d = Eigen::Array2d;
using Array3d = Eigen::Array3d;
using Array4d = Eigen::Array4d;
using Array5d = Eigen::Array<double, 5, 1>;
using Array6d = Eigen::Array<double, 6, 1>;
using Array8d = Eigen::Array<double, 8, 1>;

using Isometry3d = Eigen::Isometry3d;

};  // namespace reprojection
