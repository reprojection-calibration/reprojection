#pragma once

#include "spline/spline_state.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::calibration {

std::tuple<std::tuple<Matrix3d, CeresState>, Vector3d> EstimateCameraImuRotationAndGravity(
    Frames const& camera_poses, ImuMeasurements const& imu_data);

/**
 * \brief Estimate the approximate extrinsic rotation matrix between the IMU and camera optical frame (R_co_imu).
 *
 * Given the camera orientation spline we can differentiate it to get the camera's angular velocity (omega_co). Assuming
 * zero translation between the camera and IMU we can then use optimization::AngularVelocityAlignment() to estimate the
 * rotation matrix which aligns the camera's angular velocity to the IMU gyroscope angular velocity. Of course for any
 * real sensor pair there is translation and this will introduce an error, but for an initialization this method is
 * acceptable.
 *
 * Note that if not all axes of the camera-IMU have sufficient rotational velocity excitement then the returned solution
 * will be degenerate.
 */
std::tuple<Matrix3d, CeresState> EstimateCameraImuRotation(spline::CubicBSplineC3 const& camera_orientation_spline,
                                                           ImuMeasurements const& imu_data);

/**
 * \brief Estimate gravity in the camera's world frame using the "zero mean acceleration" assumption.
 *
 * Given a "normal" calibration sequence of roughly symmetric axis-exciting motions (i.e. up-down, twist clockwise then
 * counterclockwise, etc.), focusing on a single stationary calibration target board, the net motion-induced
 * acceleration will be zero.
 *
 * This function takes the entire set of imu linear acceleration data and transforms it into the camera's world frame
 * using the camera_orientation_spline and R_imu_co, and then sums it up. The "zero mean acceleration" principle means
 * that the acceleration that is left over after the summing is the contribution from gravity. Of course in real world
 * data with noise and biases this will never be perfect, however, we can use this value to initialize the full
 * camera-imu extrinsic calibration.
 *
 * Note that if there is no gravity present (ex. the imu prefilters out gravity) then a zero vector is returned. See the
 * return statement to understand this condition better, and assess long term if this is the right strategy.
 */
Vector3d EstimateGravity(spline::CubicBSplineC3 const& camera_orientation_spline, ImuMeasurements const& imu_data,
                         Matrix3d const& R_imu_co);

}  // namespace reprojection::calibration
