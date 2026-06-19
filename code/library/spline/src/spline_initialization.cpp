#include "spline/spline_initialization.hpp"

#include "cubic_spline_c3_init.hpp"
#include "sparse_utilities.hpp"

namespace reprojection::spline {

// TODO(Jack): Unit test!
// TODO(Jack): Rename frequency to sample_rate_hz? And change type to double?
std::pair<Matrix2NXd, TimeHandler> InitializeSe3SplineState(Frames const& frames, int const frequency) {
    PositionMeasurements so3;
    PositionMeasurements r3;
    for (auto const& [timestamp_ns, frame_i] : frames) {
        so3.insert({timestamp_ns, {frame_i.pose.topRows<constants::states>()}});
        r3.insert({timestamp_ns, {frame_i.pose.bottomRows<constants::states>()}});
    }

    double const delta_t_s{(std::crbegin(frames)->first - std::cbegin(frames)->first) / 1e9};
    int64_t const num_segments{static_cast<int64_t>(frequency * delta_t_s)};

    auto const [so3_control_points, time_handler_a]{InitializeC3SplineState(so3, num_segments)};
    auto const [r3_control_points, time_handler_b]{InitializeC3SplineState(r3, num_segments)};

    if (time_handler_a != time_handler_b) {
        throw std::runtime_error(
            "During se3 spline initialization we somehow got two different time handlers!");  // LCOV_EXCL_LINE
    }

    Matrix2NXd se3_control_points{2 * constants::states, so3_control_points.cols()};
    se3_control_points.topRows<constants::states>() = so3_control_points;
    se3_control_points.bottomRows<constants::states>() = r3_control_points;

    return {se3_control_points, time_handler_a};
}

}  // namespace reprojection::spline