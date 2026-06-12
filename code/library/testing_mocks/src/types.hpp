#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

struct Trajectory {
    Vector3d origin_w;
    Vector3d target_w;
    double radius;

};

}  // namespace reprojection::testing_mocks