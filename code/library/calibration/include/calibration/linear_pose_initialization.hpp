#pragma once

#include <memory>
#include <set>

#include "projection_functions/camera_model.hpp"
#include "types/sensor_types.hpp"

namespace reprojection::calibration {

// TODO(Jack): Is it proper here that we introduce stamped types? Or can we avoid the dependency on stamps for longer?
//  We are approaching real application level logic abstraction at this point so I think having stamped data is ok.
// TODO(Jack): The word "linear" comes more from the high order abstraction concept of the role this functions plays
//  in the top level calibration process. As the linear solution to initialize the nonlinear optimization problem.
//  Therefore it feels like the name here is wrong and should not depend on a context from a higher level of
//  abstraction. If we can think of a better name we can fix this!
std::set<PoseStamped> LinearPoseInitialization(std::set<ExtractedTargetStamped> const& targets,
                                               std::unique_ptr<projection_functions::Camera const> const& camera);

}  // namespace reprojection::calibration
