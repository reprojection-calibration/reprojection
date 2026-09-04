#pragma once

#include "optimization/bundle_adjustment_types.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

#include "bundle_adjustment_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): This function has scarily many parameters! Is this a problem or sign of a bad design?
BundleAdjustment::Result BundleAdjust(BundleAdjustment::Problem const& ba_problem, int num_threads);

ReprojectionErrors ReprojectionError(CameraInfo const& sensor, CameraMeasurements const& targets,
                                     OptimizationState const& state);

BundleAdjustment::Problem BuildSingleCamBaProblem(CameraInfo const& camera_info, CameraState const& intrinsics,
                                                  Frames const& frames, CameraMeasurements const& targets,
                                                  bool optimize_intrinsic = true);

}  // namespace  reprojection::optimization
