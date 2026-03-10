#pragma once

#include "types/algorithm_types.hpp"

namespace reprojection::calibration {

std::vector<double> InitializeFocalLengthParabolaLine(ExtractedTarget const& target, Vector2d const& principal_point);

std::vector<double> InitializeFocalLengthVanishingPoint(ExtractedTarget const& target);

}  // namespace reprojection::calibration
