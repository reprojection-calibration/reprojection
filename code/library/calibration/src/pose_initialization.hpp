#pragma once

#include "projection_functions/camera_model.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::calibration {

// TODO(Jack): Test using the mvg data generator
std::optional<Pose> EstimatePoseViaPinholePnP(std::unique_ptr<projection_functions::Camera> const& camera,
                                                    Bundle const& target, ImageBounds const& bounds);

}  // namespace reprojection::calibration
