#pragma once

#include "projection_functions/camera_model.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::calibration {

std::optional<std::pair<FrameState, double>> EstimatePoseViaPinholePnP(
    std::unique_ptr<projection_functions::Camera> const& camera, Bundle const& target, ImageBounds const& bounds);

}  // namespace reprojection::calibration
