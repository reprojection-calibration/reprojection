#pragma once

#include <vector>

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): What is our initialization strategy for real pinhole cameras?
// NOTE(Jack): We are hardcoding that fact that the intrinsics are the same for all cameras! I.e. not that every image
// could have another camera.
std::tuple<std::vector<Isometry3d>, ArrayXd> NonlinearRefinement(std::vector<Frame> const& frames,
                                                                 CameraModel const& camera_type,
                                                                 ArrayXd const& intrinsics);

}  // namespace  reprojection::optimization
