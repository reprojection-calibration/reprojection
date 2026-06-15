#pragma once

#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

struct TrajectoryParams {
    Vector3d origin_w;  // The center of the trajectory
    Vector3d target_w;  // The "look at" point for all frames in the trajectory
    double radius;
};

}  // namespace reprojection::testing_mocks