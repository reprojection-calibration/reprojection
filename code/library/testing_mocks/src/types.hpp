#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

struct Trajectory {
    Vector3d world_origin;
    double sphere_radius;
    Vector3d trajectory_center;
};

}  // namespace reprojection::testing_mocks