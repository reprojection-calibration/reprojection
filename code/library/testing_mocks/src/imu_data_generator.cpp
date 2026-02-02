#include "testing_mocks/imu_data_generator.hpp"

#include "spline/se3_spline.hpp"

#include "constants.hpp"
#include "sphere_trajectory.hpp"

namespace reprojection::testing_mocks {

// TODO PASS num_measurements OR num_control_points
// WARN(Jack): If you want to sample 100 camera poses or 100 imu measurements, then you should have at least twice as
// many control points in that spline. Otherwise you will be "oversampled" and see artifacts in the generated data.
spline::Se3Spline TimedSphereTrajectorySpline(int const num_control_points, uint64_t const duration_ns) {
    uint64_t const delta_t_ns{duration_ns / num_control_points};
    spline::Se3Spline se3_spline{0, delta_t_ns};

    for (auto const& pose : SphereTrajectory(num_control_points, constants::trajectory)) {
        se3_spline.AddControlPoint(pose);
    }

    return se3_spline;
}

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
}

ImuData GenerateImuData(int const num_samples, uint64_t const timespan_ns) {
    // TODO(Jack): Is two times num_samples really dense enough to prevent sampling artifacts?
    spline::Se3Spline const se3_spline{TimedSphereTrajectorySpline(2 * num_samples, timespan_ns)};
    std::set<uint64_t> const times{SampleTimes(num_samples, timespan_ns)};

    ImuData data;
    for (auto const time_i: times) {
        auto const velocity_t{se3_spline.Evaluate(time_i, spline::DerivativeOrder::First)};
        auto const acceleration_t{se3_spline.Evaluate(time_i, spline::DerivativeOrder::Second)};

        if (not(velocity_t.has_value() and acceleration_t.has_value())) {
            throw std::runtime_error("GenerateImuData() failed se3_spline.Evaluate().");  // LCOV_EXCL_LINE
        }

        data[time_i] = {velocity_t->topRows(3), acceleration_t->bottomRows(3)};
    }

    return data;
}

}  // namespace reprojection::testing_mocks