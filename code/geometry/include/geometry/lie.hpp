#pragma once

#include <Eigen/Dense>

namespace reprojection::geometry {

Eigen::Matrix3d Exp(Eigen::Vector3d const& so3);

Eigen::Vector3d Log(Eigen::Matrix3d const& SO3);

}  // namespace reprojection::geometry