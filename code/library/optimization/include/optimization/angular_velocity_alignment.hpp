#pragma once

#include <tuple>

#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

namespace reprojection::optimization {

/**
 * \brief Given two correspondent sets of angular velocities optimize the rotation matrix which minimizes their element
 * wise difference (i.e. minimize {omega_a - R_a_b * omega_b} as a function of R_a_b).
 *
 * This function is technically only valid when both angular velocities are measured at the same location on the
 * rotating body (i.e. the camera and imu share the exact same location). Of course for real sensor data this is not
 * actually possible. That being said we still use this function for initializing the camera-imu orientation under the
 * assumption that for small translations between the center of rotations (ex. the 10cm between drone flight controller
 * and camera) the error will be tolerable. As this is only used to initialize the full camera-imu extrinsic
 * optimization it is acceptable to have some error.
 *
 * If there is no angular velocity, or not all three axes are sufficiently excited, then the problem will be
 * underconstrained and produce a degenerate result.
 */
std::tuple<Matrix3d, CeresState> AngularVelocityAlignment(VelocityMeasurements const& omega_a,
                                                          VelocityMeasurements const& omega_b);

}  // namespace  reprojection::optimization
