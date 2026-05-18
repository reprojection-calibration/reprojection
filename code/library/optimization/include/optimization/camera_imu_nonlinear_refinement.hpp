#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

std::tuple<std::pair<CameraState, spline::Se3Spline>, Matrix3d, CeresState> SplineNonlinearRefinement(
    CameraInfo const& sensor, CameraMeasurements const& targets, ImuMeasurements const& imu_data,
    CameraState const& camera_state, Matrix3d const& R_imu_co, spline::Se3Spline const& spline);

// TODO NAMING AND LOCATION
// TODO SPLINE TYPE!!!
std::pair<Frames, ReprojectionErrors> SplineReprojectionResiduals(CameraInfo const& sensor,
                                                                  CameraMeasurements const& targets,
                                                                  CameraState const& camera_state,
                                                                  spline::Se3Spline const& spline);

}  // namespace  reprojection::optimization
