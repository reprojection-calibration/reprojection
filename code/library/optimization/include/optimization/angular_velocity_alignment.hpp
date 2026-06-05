#pragma once

#include "spline/se3_spline.hpp"
#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

namespace reprojection::optimization {

// NOTE(Jack): We actually only need to pass the rotation component of the spline, but because of the way that ceres
// works and how we use the rigid body angular velocity cost function we have to pass in the full size control points so
// we pass the full spline here.

/**
 * \brief Estimate the approximate extrinsic rotation matrix between the IMU and camera optical frame (R_co_imu).
 *
 * Given the camera orientation spline we can differentiate it to get the camera's angular velocity (omega_co). Because
 * all points on a rigid body have the same angular velocity we can estimate the rotation matrix which aligns the
 * camera's angular velocity to the IMU's angular velocity.
 *
 * Note that if not all axes of the camera-IMU motion have sufficient rotational velocity excitement then the returned
 * solution will be degenerate.
 */
std::pair<Array3d, CeresState> AngularVelocityAlignment(VelocityMeasurements const& omega_imu,
                                                        spline::Se3Spline const& spline);

}  // namespace  reprojection::optimization
