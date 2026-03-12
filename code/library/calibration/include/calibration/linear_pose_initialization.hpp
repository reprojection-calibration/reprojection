#pragma once

#include "projection_functions/camera_model.hpp"
#include "types/calibration_types.hpp"

namespace reprojection::calibration {

// NOTE(Jack): The word "linear" comes more from the high order abstraction concept of the role this functions plays
//  in the top level calibration process. As the linear solution to initialize the nonlinear optimization problem.
//  Therefore it feels like the name here is wrong and should not depend on a context from a higher level of
//  abstraction. If we can think of a better name we can fix this!
Frames LinearPoseInitialization(CameraInfo const& sensor, CameraMeasurements const& targets,
                                CameraState const& intrinsics);

std::optional<std::pair<FrameState, double>> EstimatePoseViaPinholePnP(
    std::unique_ptr<projection_functions::Camera> const& camera, Bundle const& target, ImageBounds const& bounds);

}  // namespace reprojection::calibration
