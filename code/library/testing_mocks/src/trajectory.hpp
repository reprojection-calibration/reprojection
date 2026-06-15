#pragma once

#include "geometry/lie.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_mocks {

std::pair<Frames, ImuMeasurements> Trajectory(double const duration_s, double const sample_rate_hz,
                                               Vector3d const& origin_w, Vector3d const& target_w, double const radius);

Vector3d PositionWorldBody(uint64_t const timestamp_ns, Vector3d const& origin_w, double const radius);

Matrix3d LookAtRotationWorldBody(Vector3d const& position_w, Vector3d const& target_w,
                                 std::optional<Matrix3d> const& R_w_b_prev);

Eigen::Array<uint64_t, -1, 1> SampleTimes(double const duration_s, double const sample_rate_hz);



Matrix3d RollAboutBodyX(uint64_t const timestamp_ns);

inline Matrix3d RotY(double const theta) { return geometry::Exp<double>({0, theta, 0}); }

inline Matrix3d RotZ(double const theta) { return geometry::Exp<double>({0, 0, theta}); }

inline Vector3d Vee(Matrix3d const& mat) { return Vector3d{mat(2, 1), mat(0, 2), mat(1, 0)}; }

}  // namespace reprojection::testing_mocks