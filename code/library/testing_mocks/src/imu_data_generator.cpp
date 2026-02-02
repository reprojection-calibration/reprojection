#include "testing_mocks/imu_data_generator.hpp"

#include "spline/se3_spline.hpp"

#include "data_generator_helpers.hpp"

namespace reprojection::testing_mocks {

ImuData GenerateImuData(int const num_samples, uint64_t const timespan_ns) {
    // TODO(Jack): Is two times num_samples really dense enough to prevent sampling artifacts?
    spline::Se3Spline const se3_spline{TimedSphereTrajectorySpline(2 * num_samples, timespan_ns)};
    std::set<uint64_t> const times{SampleTimes(num_samples, timespan_ns)};

    ImuData data;
    for (auto const time_i : times) {
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