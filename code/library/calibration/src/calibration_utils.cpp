#include "calibration/calibration_utils.hpp"

#include <ranges>

namespace reprojection::calibration {

// WARN(Jack): This is a hack that we need to do so that the spline initialization does not have any massive
// discontinuities or sudden jumps. But there is some bigger problem here that we are missing and need to solve long
// term.
Frames AlignRotations(Frames data) {
    if (std::empty(data)) {
        return data;
    }

    Vector3d so3_i_1{std::cbegin(data)->second.value.head<3>()};
    for (auto& frame_i : data | std::views::values) {
        Vector3d so3_i{frame_i.value.head<3>()};
        double const dp{so3_i_1.dot(so3_i)};

        if (dp < 0) {
            so3_i *= -1.0;
        }
        frame_i.value.head<3>() = so3_i;

        so3_i_1 = so3_i;
    }

    return data;
}

}  // namespace reprojection::calibration
