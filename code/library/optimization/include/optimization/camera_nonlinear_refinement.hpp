#pragma once

#include <tuple>

#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

std::tuple<OptimizationState, CeresState> CameraNonlinearRefinement(CameraInfo const& sensor,
                                                                    CameraMeasurements const& targets,
                                                                    OptimizationState const& initial_state);

ReprojectionErrors ReprojectionResiduals(CameraInfo const& sensor, CameraMeasurements const& targets,
                                         OptimizationState const& state);

}  // namespace  reprojection::optimization
