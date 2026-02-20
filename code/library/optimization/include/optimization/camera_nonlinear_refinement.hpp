#pragma once

#include <ceres/ceres.h>

#include <vector>

#include "optimization/ceres_state.hpp"
#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// NOTE(Jack): We are hardcoding that fact that the intrinsics are the same for all cameras! I.e. not that every image
// could have another camera. Each data view is a view into one single camera so this makes sense!
std::tuple<OptimizationState, CeresState> CameraNonlinearRefinement(CameraInfo const& sensor,
                                                                    CameraMeasurements const& targets,
                                                                    OptimizationState const& initial_state);

ReprojectionErrors ReprojectionResiduals(CameraInfo const& sensor, CameraMeasurements const& targets,
                                         OptimizationState const& state);

}  // namespace  reprojection::optimization
