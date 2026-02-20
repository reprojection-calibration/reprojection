#include "testing_mocks/imu_data_generator.hpp"

#include "spline/se3_spline.hpp"

#include "data_generator_helpers.hpp"

namespace reprojection::testing_mocks {

ImuData GenerateImuData(int const num_samples, uint64_t const timespan_ns) {
    spline::Se3Spline const trajectory{TimedSphereTrajectorySpline(5 * num_samples, timespan_ns)};
    std::set<uint64_t> const times{SampleTimes(num_samples, timespan_ns)};

    ImuData data;
    for (auto const time_i : times) {
        auto const velocity_t{trajectory.Evaluate(time_i, spline::DerivativeOrder::First)};
        auto const acceleration_t{trajectory.Evaluate(time_i, spline::DerivativeOrder::Second)};

        if (not(velocity_t.has_value() and acceleration_t.has_value())) {
            throw std::runtime_error("GenerateImuData() failed trajectory.Evaluate().");  // LCOV_EXCL_LINE
        }

        // TODO(Jack): Kind of hacky that we store the measurement data which has timestamp in a timestamped map. See
        // comment at tope of method.
        data[time_i] = {time_i, velocity_t->topRows(3), acceleration_t->bottomRows(3)};
    }

    return data;
}

}  // namespace reprojection::testing_mocks