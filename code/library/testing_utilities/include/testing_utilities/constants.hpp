#pragma once

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::testing_utilities {

// TODO(Jack): For the spline constants I googled that the best way to make a global const value was via "inline
// constexpr". If this is true or actually important that we do it I am not sure.
inline constexpr ImageBounds image_bounds{0, 720, 0, 480};

inline constexpr ImageBounds unit_image_bounds{-1, 1, -1, 1};

// NOTE(Jack): Eigen cannot work with constexpr :(
inline const Array4d pinhole_intrinsics{600, 600, 360, 240};

inline const Array4d unit_pinhole_intrinsics{1, 1, 0, 0};

}  // namespace reprojection::testing_utilities