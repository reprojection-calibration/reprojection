#pragma once

#include "projection_functions/camera_model.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::calibration {

// TODO(Jack): Is this really the best name we got? Are we sure this is describing the code in the most accurate way?
std::optional<std::pair<FrameState, double>> EstimatePoseViaPinholePnP(
    std::unique_ptr<projection_functions::Camera> const& camera, Bundle const& target, ImageBounds const& bounds);

}  // namespace reprojection::calibration
