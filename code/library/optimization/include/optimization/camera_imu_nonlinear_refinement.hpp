#pragma once

#include "spline/se3_spline.hpp"
#include "types/calibration_types.hpp"
#include "types/ceres_types.hpp"

namespace reprojection::optimization {

// TODO NAMING
std::tuple<std::pair<CameraState, spline::Se3Spline>, CeresState> SplineNonlinearRefinement(CameraInfo const& sensor,
                                                                    CameraMeasurements const& targets,
                                                                    CameraState const& camera_state,
                                                                    spline::Se3Spline const& spline);

// TODO NAMING AND LOCATION
// TODO SPLINE TYPE!!!
std::pair<Frames, ReprojectionErrors> SplineReprojectionResiduals(CameraInfo const& sensor,
                                                                  CameraMeasurements const& targets,
                                                                  CameraState const& camera_state,
                                                                  spline::Se3Spline const& spline);

}  // namespace  reprojection::optimization
