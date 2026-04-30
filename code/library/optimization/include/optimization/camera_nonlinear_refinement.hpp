#pragma once

#include <tuple>

#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This function has scarily many parameters! Is this a problem or sign of a bad design?
std::tuple<OptimizationState, CeresState> CameraNonlinearRefinement(CameraInfo const& sensor,
                                                                    CameraMeasurements const& targets,
                                                                    OptimizationState const& initial_state,
                                                                    bool const constant_intrinsics = false);

ReprojectionErrors ReprojectionResiduals(CameraInfo const& sensor, CameraMeasurements const& targets,
                                         OptimizationState const& state);

}  // namespace  reprojection::optimization
