#pragma once

#include "types.hpp"

namespace reprojection::testing_mocks::constants {

// WARN(Jack): This be getting a little hacky/globally, but hey it's at least const :) But the reason we need this is
// because we need to have the same trajectory geometry for both the mvg_generator and imu_generator. If the best way to
// enforce this is via constants here is not clear. But for now it is a solution.
Trajectory const trajectory{{1.5, 1.5, 1.5}, {0, 0, 0}, 1};

}  // namespace reprojection::testing_mocks::constants