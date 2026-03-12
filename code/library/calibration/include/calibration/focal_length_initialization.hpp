#pragma once

#include <ranges>

#include "projection_functions/double_sphere.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"

namespace reprojection::calibration {

// TODO NAMING!!!!
using GammaRunner = std::function<std::vector<double>(ExtractedTarget const&)>;
using FocalLengthInit = std::function<ArrayXd(double, double, double)>;

std::pair<GammaRunner, FocalLengthInit> RuntimeInitializationDispatch(CameraModel const camera_model,
                                                                      double const height, double const width);

std::vector<double> InitializeFocalLengthParabolaLine(ExtractedTarget const& target, double const cx, double const cy);

}  // namespace reprojection::calibration
