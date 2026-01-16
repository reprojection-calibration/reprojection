#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace reprojection {

// NOTE(Jack): Numbers come before letters in alphabetical order!

template <typename T>
using Array2 = Eigen::Array<T, 2, 1>;
using Array2d = Eigen::Array2d;
using Array2i = Array2<int>;
template <typename T>
using Array3 = Eigen::Array<T, 3, 1>;
using Array3d = Array3<double>;
using Array4d = Eigen::Array4d;
using Array4i = Eigen::Array4i;
template <typename T>
using Array5 = Eigen::Array<T, 5, 1>;
using Array5b = Array5<bool>;
using Array5d = Array5<double>;
using Array6d = Eigen::Array<double, 6, 1>;
using Array8d = Eigen::Array<double, 8, 1>;
template <typename T>
using ArrayX = Eigen::Array<T, Eigen::Dynamic, 1>;
using ArrayXb = ArrayX<bool>;
using ArrayXd = ArrayX<double>;
using ArrayXi = ArrayX<int>;
using ArrayX2d = Eigen::ArrayX2d;
using ArrayX2i = Eigen::ArrayX2i;

using Matrix2d = Eigen::Matrix2d;
template <typename T>
using Matrix3 = Eigen::Matrix3<T>;
using Matrix3d = Matrix3<double>;
using Matrix34d = Eigen::Matrix<double, 3, 4>;
using Matrix42d = Eigen::Matrix<double, 4, 2>;
using Matrix4d = Eigen::Matrix4d;
using MatrixX2d = Eigen::MatrixX2d;
using MatrixX3d = Eigen::MatrixX3d;
using MatrixX4d = Eigen::MatrixX4d;
using MatrixXd = Eigen::MatrixXd;
using MatrixXi = Eigen::MatrixXi;

using Vector2d = Eigen::Vector2d;
using Vector2i = Eigen::Vector2i;
template <typename T>
using Vector3 = Eigen::Vector3<T>;
using Vector3d = Vector3<double>;
using Vector4d = Eigen::Vector4d;
using Vector6d = Eigen::Vector<double, 6>;
using VectorXd = Eigen::VectorXd;

// Miscellaneous types
using Isometry3d = Eigen::Isometry3d;

};  // namespace reprojection
