#pragma once

#include <cstdint>

#include "types.hpp"

namespace reprojection::testing_mocks::constants {

// Number of "loops" in the sphere trajectory to generate.
int const num_loops{4};

// Starting time of the SE3 spline
std::uint64_t const t0_ns{0};
// Time increment between each control point (sphere trajectory pose) of the SE3 spline
std::uint64_t const delta_t_ns{1000000};  // 1ms

// WARN(Jack): This be getting a little hacky/globally, but hey it's at least const :) But the reason we need this is
// because we need to have the same trajectory geometry for both the mvg_generator and imu_generator. If the best way to
// enforce this is via constants here is not clear. But for now it is a solution.
CameraTrajectory const trajectory{{0, 0, 0}, 0.4, {0, 0, 2}};

}  // namespace reprojection::testing_mocks::constants