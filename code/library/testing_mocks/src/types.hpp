#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

struct CameraTrajectory {
    Vector3d world_origin;
    double sphere_radius;
    Vector3d sphere_origin;
};

}  // namespace reprojection::testing_mocks