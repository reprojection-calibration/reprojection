#pragma once

#include <optional>

#include "types/eigen_types.hpp"

namespace reprojection::calibration {

// principal_point is provided in pixel coordinates and pixels is at least four pixels from four 3D features which lie
// on a straight line (i.e. the row/column of a calibration pattern.)
std::optional<double> ParabolaLineInitialization(Vector2d const& principal_point, MatrixX2d const& pixels);

}  // namespace reprojection::calibration
