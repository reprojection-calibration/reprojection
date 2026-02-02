#pragma once

#include <set>

#include "spline/se3_spline.hpp"

namespace reprojection::testing_mocks {

// TODO TEST
// WARN(Jack): If you want to sample 100 camera poses or 100 imu measurements, then you should have at least twice as
// many control points in that spline. Otherwise you will be "oversampled" and see artifacts in the generated data.
spline::Se3Spline TimedSphereTrajectorySpline(int const num_control_points, uint64_t const duration_ns);

// TODO TEST
std::set<uint64_t> SampleTimes(int const num_samples, uint64_t const timespan_ns);

}  // namespace reprojection::testing_mocks