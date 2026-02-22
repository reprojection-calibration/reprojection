#pragma once

#include <tuple>

#include "optimization/ceres_state.hpp"
#include "types/spline_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

std::tuple<Matrix3d, CeresState> InitializeCameraImuOrientation(VelocityMeasurements const& omega_co,
                                                                  VelocityMeasurements const& omega_imu);

}  // namespace  reprojection::optimization
