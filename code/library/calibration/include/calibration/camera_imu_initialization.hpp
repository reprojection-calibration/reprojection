#pragma once

#include "spline/spline_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

namespace reprojection::calibration {

std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> EstimateCameraImuRotationAndGravity(
    spline::CubicBSplineC3 const& camera_orientation_spline, ImuMeasurements const& imu_data);

/**
 * \brief Estimate the approximate extrinsic rotation matrix between the IMU and camera optical frame (R_co_imu).
 *
 * Given the camera orientation spline we can differentiate it to get the camera's angular velocity (omega_co). Because
 * all points on a rigit body have the same angular velocity we can use optimization::AngularVelocityAlignment() to
 * estimate the rotation matrix which aligns the camera's angular velocity to the IMU gyroscope's angular velocity.
 *
 * Note that if not all axes of the camera-IMU motion have sufficient rotational velocity excitement then the returned
 * solution will be degenerate.
 */
std::tuple<Matrix3d, CeresState> EstimateCameraImuRotation(spline::CubicBSplineC3 const& camera_orientation,
                                                           VelocityMeasurements const& omega_imu);

/**
 * \brief Estimate gravity in the camera's world frame using the "zero mean acceleration" assumption.
 *
 * Given a "normal" calibration sequence of roughly symmetric axis-exciting motions (i.e. up-down, twist clockwise then
 * counterclockwise, etc.), focusing on a single stationary calibration target board, the net motion-induced
 * acceleration is zero.
 *
 * This function takes the entire set of imu linear acceleration data and transforms it into the camera's world frame
 * using the camera_orientation spline and R_imu_co. Once in the world from it sums up each component of the
 * acceleration (x,y,z). The "zero mean acceleration" principle means that the acceleration that is left over after the
 * summing is the contribution from gravity (i.e. the only asymmetric acceleration present). Of course in real world
 * data with noise and biases this will never be perfect, however, we can use this value to initialize the full
 * camera-imu extrinsic optimization.
 *
 * The transformation of the IMU's linear acceleration to the camera frame requires the assumption of zero translation.
 * This is not possible because two sensors cannot be at the exact same location, but the resulting error is acceptable
 * for an initialization method. Assuming of course the translation is not too large :)
 *
 * Note that if there is no gravity present (ex. the imu prefilters out gravity) then a zero vector is returned. See the
 * return statement to understand this condition better, and assess long term if this is the right strategy.
 */
Vector3d EstimateGravity(spline::CubicBSplineC3 const& camera_orientation,
                         AccelerationMeasurements const& acceleration_imu, Matrix3d const& R_imu_co);

}  // namespace reprojection::calibration
