#pragma once

#include "types/calibration_types.hpp"

namespace reprojection::calibration {

// TODO(Jack): Is the name too generic?
enum class InitializationMethod { ParabolaLine, VanishingPoint };

double InitializeFocalLength(ExtractedTarget const& target, InitializationMethod const method);

// TODO(Jack): Probably does not need to be part of the public api
std::tuple<std::vector<Bundle>, std::vector<Bundle>> SortIntoRowsAndCols(ExtractedTarget const& target);

}  // namespace reprojection::calibration