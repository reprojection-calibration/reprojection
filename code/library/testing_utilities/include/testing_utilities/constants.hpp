#pragma once

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_utilities {

// TODO(Jack): For the spline constants I googled that the best way to make a global const value was via "inline
// constexpr". If this is true or actually important that we do it I am not sure.
inline ImageBounds constexpr image_bounds{0, 720, 0, 480};

inline ImageBounds constexpr unit_image_bounds{-1, 1, -1, 1};

// NOTE(Jack): Eigen cannot work with constexpr :(
inline Array4d const pinhole_intrinsics{600, 600, 360, 240};

inline Array4d const unit_pinhole_intrinsics{1, 1, 0, 0};

// NOTE(Jack): At time of writing these are only used in the projection_functions module, so maybe they belong there in
// that smaller scope rather than here at the global level. But for now we do it like this.
inline MatrixX3d const gt_points{{0, 0, 600},  //
                                 {-360, 0, 600},
                                 {359.9, 0, 600},
                                 {0, -240, 600},
                                 {0, 239.9, 600}};

}  // namespace reprojection::testing_utilities