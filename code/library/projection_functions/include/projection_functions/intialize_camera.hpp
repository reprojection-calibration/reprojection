#pragma once

#include "projection_functions/camera_model.hpp"

namespace reprojection::projection_functions {

// TODO(Jack): Add all other camera models and check the above listed TODO points.
// TODO(Jack): Test!
std::unique_ptr<Camera> InitializeCamera(CameraModel const model, ArrayXd const& intrinsics, ImageBounds const& bounds);

}  // namespace reprojection::projection_functions