#pragma once

#include <Eigen/Dense>
#include <optional>
#include <tuple>

namespace reprojection::calibration {

using Circle = std::tuple<Eigen::Vector2d, double>;

// Adopted from https://stackoverflow.com/questions/3349125/circle-circle-intersection-points which copy and pasted from
// here https://paulbourke.net/geometry/circlesphere/
std::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>> CircleCircleIntersection(Circle const& c1,
                                                                                     Circle const& c2);

std::optional<Circle> FitCircle(Eigen::MatrixX2d const& data);

}  // namespace reprojection::calibration