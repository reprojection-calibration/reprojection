#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::pnp {

bool IsPlane(MatrixX3d const& points);

// TODO(Jack): Not tested
std::tuple<Vector3d, Matrix3d> Pca(MatrixX3d const& points);

}  // namespace reprojection::pnp