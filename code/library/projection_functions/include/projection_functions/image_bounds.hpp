#pragma once

#include "types/calibration_types.hpp"

namespace reprojection::projection_functions {

template <typename T>
bool InBounds(ImageBounds const& bounds, T const u, T const v) {
    return bounds.u_min < u and u < bounds.u_max and bounds.v_min < v and v < bounds.v_max;
}

}  // namespace reprojection::projection_functions