#pragma once

#include <optional>
#include <tuple>

#include "types/eigen_types.hpp"

namespace reprojection::calibration {

// (center: (x,y), radius: double)
using Circle = std::tuple<Vector2d, double>;

// Follows the method from "Equidistant Fish-Eye Calibration and Rectification by Vanishing Point Extraction" Ciara´n
// Hughes, Patrick Denny, Martin Glavin, and Edward Jones
std::optional<double> VanishingPointInitialization(MatrixX2d const& pixels1, MatrixX2d const& pixels2);

// Adopted from https://stackoverflow.com/questions/3349125/circle-circle-intersection-points which copy and pasted from
// here https://paulbourke.net/geometry/circlesphere/
std::optional<std::tuple<Vector2d, Vector2d>> CircleCircleIntersection(Circle const& c1, Circle const& c2);

// Modified Least Squares method (MLS) from "A Few Methods for Fitting Circles to Data" Dale Umbach, Kerry N. Jones
std::optional<Circle> FitCircle(MatrixX2d const& data);

}  // namespace reprojection::calibration