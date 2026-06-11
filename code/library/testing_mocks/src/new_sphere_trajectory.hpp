#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

Vector3d TrajectoryPosition(uint64_t const timestamp_ns, Vector3d const origin_w, double const radius);

Matrix3d LookAtRotationWorldBody(Vector3d const position_w, Vector3d const target_w);

}  // namespace reprojection::testing_mocks