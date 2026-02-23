#pragma once

#include <tuple>

#include "types/ceres_types.hpp"
#include "types/eigen_types.hpp"
#include "types/spline_types.hpp"

namespace reprojection::optimization {

std::tuple<Matrix3d, CeresState> AngularVelocityAlignment(VelocityMeasurements const& omega_co,
                                                                VelocityMeasurements const& omega_imu);

}  // namespace  reprojection::optimization
