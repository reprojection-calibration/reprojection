#pragma once

#include <Eigen/Dense>
#include <optional>

namespace reprojection::calibration {

std::tuple<Eigen::Vector2d, double> FitCircle(Eigen::MatrixX2d const& data);

}  // namespace reprojection::calibration