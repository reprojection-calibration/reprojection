#pragma once

#include <vector>

#include "types/eigen_types.hpp"

namespace reprojection::geometry {

Isometry3d Exp(Vector6d const& se3);

Vector6d Log(Isometry3d const& SE3);

Matrix3d Exp(Vector3d const& so3);

Vector3d Log(Matrix3d const& SO3);

std::vector<Isometry3d> ToSE3(std::vector<Array6d> const& se3);

}  // namespace reprojection::geometry