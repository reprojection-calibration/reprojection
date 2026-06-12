#pragma once

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

// TODO MAKE PRIVATE!

namespace reprojection::testing_mocks {

Vector3d TrajectoryPosition(uint64_t const timestamp_ns, Vector3d const& origin_w, double const radius);

Matrix3d LookAtRotationWorldBody(Vector3d const& position_w, Vector3d const& target_w,
                                 std::optional<Matrix3d> const& R_w_b_prev);

Eigen::Array<uint64_t, -1, 1> SampleTimes(double const duration_s, double const sample_rate_hz);

std::pair<Frames, ImuMeasurements> Trajectory2(double const duration_s, double const sample_rate_hz,
                                               Vector3d const& origin_w, Vector3d const& target_w, double const radius);

}  // namespace reprojection::testing_mocks