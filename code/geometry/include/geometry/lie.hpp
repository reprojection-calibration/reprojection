#pragma once

#include <vector>

#include "types/eigen_types.hpp"

namespace reprojection::geometry {

Isometry3d Exp(Vector6d const& se3);

Vector6d Log(Isometry3d const& SE3);

template <typename T>
Eigen::Matrix3<T> Exp(Eigen::Vector3<T> const& so3) {
    Eigen::Matrix3<T> const SO3{Eigen::AngleAxis<T>(so3.norm(), so3.normalized()).toRotationMatrix()};

    return SO3;
}

template <typename T>
Eigen::Vector3<T> Log(Eigen::Matrix3<T> const& SO3) {
    Eigen::AngleAxis<T> const rotation(SO3);
    Eigen::Vector3<T> const so3{rotation.angle() * rotation.axis()};

    return so3;
}

std::vector<Isometry3d> ToSE3(std::vector<Array6d> const& se3);

}  // namespace reprojection::geometry