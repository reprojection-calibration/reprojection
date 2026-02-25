#include "data_generator_helpers.hpp"

#include <set>

#include "constants.hpp"
#include "sphere_trajectory.hpp"

namespace reprojection::testing_mocks {

spline::Se3Spline TimedSphereTrajectorySpline(int const num_control_points, uint64_t const duration_ns) {
    // WARN(Jack): Rounding error?
    uint64_t const delta_t_ns{duration_ns / num_control_points};

    std::vector<Vector6d> const trajectory{SphereTrajectory(num_control_points, constants::trajectory)};

    return spline::Se3Spline{spline::TimeHandler{0, delta_t_ns}, trajectory};  // Hardcoded to start at 0
}  // LCOV_EXCL_LINE

std::set<uint64_t> SampleTimes(int const num_samples, uint64_t const timespan_ns) {
    uint64_t const delta_t_ns{timespan_ns / num_samples};

    std::set<uint64_t> sample_times;
    for (int i{0}; i < num_samples; ++i) {
        double const elapsed_trajectory{static_cast<double>(i) / num_samples};
        uint64_t const spline_time_i{
            static_cast<uint64_t>((num_samples - spline::constants::degree) * delta_t_ns * elapsed_trajectory)};

        sample_times.insert(spline_time_i);
    }

    return sample_times;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::testing_mocks