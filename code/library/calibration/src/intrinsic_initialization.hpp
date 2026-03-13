#pragma once

#include "types/algorithm_types.hpp"
#include "types/calibration_types.hpp"
#include "types/enums.hpp"

namespace reprojection::calibration {

// TODO(Jack): Should we do a "initialization strategy" struct?
using CandidateGenerator = std::function<std::vector<double>(ExtractedTarget const&)>;
using IntrinsicsInitializer = std::function<ArrayXd(double, double, double)>;

std::pair<CandidateGenerator, IntrinsicsInitializer> SelectInitializationStrategy(CameraModel const camera_model,
                                                                                  double const height,
                                                                                  double const width);

std::vector<double> EstimateCandidatesParabolaLine(ExtractedTarget const& target, double const cx, double const cy);

std::vector<double> EstimateCandidatesVanishingPoint(ExtractedTarget const& target);

}  // namespace reprojection::calibration
