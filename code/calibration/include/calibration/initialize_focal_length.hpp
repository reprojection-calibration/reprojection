#pragma once

#include "types/calibration_types.hpp"

namespace reprojection::calibration {

// TODO(Jack): Is the name too generic?
enum class InitializationMethod { ParabolaLine, VanishingPoint };

// NOTE(Jack): Principle point only required for ParabolaLineInitialization, is there anyway to prevent this?
// should we pass the principal point as an optional?
std::vector<double> InitializeFocalLength(ExtractedTarget const& target, InitializationMethod const method,
                                          Vector2d const& principal_point);

// TODO(Jack): Probably does not need to be part of the public api
std::tuple<std::vector<Bundle>, std::vector<Bundle>> SortIntoRowsAndCols(ExtractedTarget const& target);

}  // namespace reprojection::calibration