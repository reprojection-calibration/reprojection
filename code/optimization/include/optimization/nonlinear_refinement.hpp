#pragma once

#include <vector>

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::optimization {

// TODO(Jack): Technically the nonlinear refinement does not need the IDs, it just need correspondent pixels and points.
// However it might be "easier" to just pass in the entire frame, including ids. However this will be too much
// information!
// TODO(Jack): What is our initialization strategy for real pinhole cameras?
// TODO(Jack): THere are a lot of parameters here! Is there any change that the three vector heres (pixels, points,
// poses) actually form one "type"? Lets see what happens and decide later!
// NOTE(Jack): We are hardcoding that fact that the intrinsics are the same for all cameras! I.e. not that every image
// could have another camera.
std::tuple<std::vector<Isometry3d>, ArrayXd> NonlinearRefinement(std::vector<Frame> const& frames,
                                                                 CameraModel const& camera_type,
                                                                 ArrayXd const& intrinsics);

}  // namespace  reprojection::optimization
