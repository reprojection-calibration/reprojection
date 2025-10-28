#pragma once

#include <optional>

#include "types/eigen_types.hpp"

namespace reprojection::calibration {

// Follows the method from "Single View Point Omnidirectional Camera Calibration from Planar Grids" Christopher Mei, P.
// Rives
std::optional<double> ParabolaLineInitialization(Vector2d const& principal_point, MatrixX2d const& pixels);

}  // namespace reprojection::calibration
