#pragma once

#include "calibration_data_views/initialization_view.hpp"

namespace reprojection::calibration {

// NOTE(Jack): The word "linear" comes more from the high order abstraction concept of the role this functions plays
//  in the top level calibration process. As the linear solution to initialize the nonlinear optimization problem.
//  Therefore it feels like the name here is wrong and should not depend on a context from a higher level of
//  abstraction. If we can think of a better name we can fix this!
void LinearPoseInitialization(InitializationDataView data_view);

}  // namespace reprojection::calibration
