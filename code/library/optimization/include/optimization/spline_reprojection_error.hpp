#pragma once

#include "spline/time_handler.hpp"
#include "spline/types.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO NAMING AND LOCATION
// TODO SPLINE TYPE!!!
ReprojectionErrors SplineReprojectionResiduals(CameraInfo const& sensor, CameraMeasurements const& targets,
                                               CameraState const& camera_state,
                                               std::pair<spline::Matrix2NXd, spline::TimeHandler> const& spline);

}  // namespace  reprojection::optimization
