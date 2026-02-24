#pragma once

#include <tuple>

#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

std::tuple<Matrix3d, CeresState> CameraImuNonlinearRefinement(CameraInfo const& sensor,
                                                              CameraMeasurements const& targets,
                                                              ImuMeasurements const& imu_data,
                                                              OptimizationState const& initial_state);

}  // namespace  reprojection::optimization
