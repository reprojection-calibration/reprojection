#pragma once

#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"

namespace reprojection::calibration {

using IntrinsicsInitializer = std::function<ArrayXd(double, double, double)>;

IntrinsicsInitializer SelectInitializationStrategy(CameraModel camera_model);

std::vector<double> EstimateCandidatesParabolaLine(ExtractedTarget const& target, double cx, double cy);

std::vector<double> EstimateCandidatesVanishingPoint(ExtractedTarget const& target);

}  // namespace reprojection::calibration
