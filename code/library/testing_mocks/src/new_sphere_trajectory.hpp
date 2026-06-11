#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

Vector3d TrajectoryPosition(uint64_t const timestamp_ns, Vector3d const origin_w, double const radius);

}  // namespace reprojection::testing_mocks