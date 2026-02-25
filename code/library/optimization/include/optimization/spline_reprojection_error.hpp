#pragma once

#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO NAMING AND LOCATION
ReprojectionErrors SplineReprojectionResiduals(CameraInfo const& sensor, CameraMeasurements const& targets,
                                           OptimizationState const& state);

}  // namespace  reprojection::optimization
