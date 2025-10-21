#pragma once

namespace reprojection::testing_mocks::constants {

// Number of poses in the sphere trajectory to generate
int const num_poses{100};
// Number of "loops" in the sphere trajectory to generate.
int const num_loops{4};

// Starting time of the SE3 spline
uint64_t const t0_ns{0};
// Time increment between each knot (sphere trajectory pose) of the SE3 spline
uint64_t const delta_t_ns{1000000};  // 1ms

}  // namespace reprojection::testing_mocks::constants