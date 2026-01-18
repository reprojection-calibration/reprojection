#pragma once

#include "types/calibration_types.hpp"

namespace reprojection::projection_functions {

// NOTE(Jack): Our current definition of image bounds is [min, max). This means that an image bounds set with a width
// dimension of 0 to 720 will consider 0 a valid dimension, but 720 will be out of bounds. The maximum valid dimension
// would be 719.999999... If this is really algorithmically accurate, is not clear at this time.
template <typename T>
bool InBounds(ImageBounds const& bounds, T const u, T const v) {
    return bounds.u_min <= u and u < bounds.u_max and bounds.v_min <= v and v < bounds.v_max;
}

}  // namespace reprojection::projection_functions