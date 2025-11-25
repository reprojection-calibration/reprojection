#pragma once

#include "types/calibration_types.hpp"

namespace reprojection::calibration {

// TODO(Jack): Is the name too generic?
enum class InitializationMethod { ParabolaLine, VanishingPoint };

double InitializeFocalLength(ExtractedTarget const& target, InitializationMethod const method);

}  // namespace reprojection::calibration