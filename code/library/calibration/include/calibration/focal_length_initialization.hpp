#pragma once

#include <ranges>

#include "projection_functions/double_sphere.hpp"
#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"

namespace reprojection::calibration {

std::optional<ArrayXd> InitializeIntrinsics(CameraModel const camera_model, double const height, double const width,
                                           CameraMeasurements const& targets);

// TODO THIS CAN BE PUT IN A PRIVATE HEADER!?
// TODO NAMING!!!!
using GammaRunner = std::function<std::vector<double>(ExtractedTarget const&)>;
using FocalLengthInit = std::function<ArrayXd(double, double, double)>;

// TODO THIS CAN BE PUT IN A PRIVATE HEADER!?
std::pair<GammaRunner, FocalLengthInit> RuntimeInitializationDispatch(CameraModel const camera_model,
                                                                      double const height, double const width);

// TODO THIS CAN BE PUT IN A PRIVATE HEADER!?
std::vector<double> InitializeFocalLengthParabolaLine(ExtractedTarget const& target, double const cx, double const cy);

}  // namespace reprojection::calibration
