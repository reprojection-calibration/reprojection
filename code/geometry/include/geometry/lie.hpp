#pragma once

#include <Eigen/Dense>

namespace reprojection::geometry {

Eigen::Isometry3d Exp(Eigen::Vector<double, 6> const& se3);

Eigen::Vector<double, 6> Log(Eigen::Isometry3d const& SE3);

Eigen::Matrix3d Exp(Eigen::Vector3d const& so3);

Eigen::Vector3d Log(Eigen::Matrix3d const& SO3);

}  // namespace reprojection::geometry