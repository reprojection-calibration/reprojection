#pragma once

#include <ceres/ceres.h>

#include <vector>

#include "calibration_data_views/optimization_view.hpp"
#include "optimization/ceres_state.hpp"

namespace reprojection::optimization {

// NOTE(Jack): We are hardcoding that fact that the intrinsics are the same for all cameras! I.e. not that every image
// could have another camera. Each data view is a view into one single camera so this makes sense!
CeresState CameraNonlinearRefinement(OptimizationDataView data_view);

// TODO(Jack): This does not need to be part of the public interface as it is used internally only as part of
//  CameraNonlinearRefinement.
ArrayX2d EvaluateReprojectionResiduals(std::vector<std::unique_ptr<ceres::CostFunction>> const& cost_functions,
                                       ArrayXd const& intrinsics, Array6d const& pose);

}  // namespace  reprojection::optimization
