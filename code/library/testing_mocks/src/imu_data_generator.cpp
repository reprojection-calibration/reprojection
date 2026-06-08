#include "testing_mocks/imu_data_generator.hpp"

#include "spline/se3_spline.hpp"

#include "data_generator_helpers.hpp"

namespace reprojection::testing_mocks {

std::pair<ImuMeasurements, spline::Se3Spline> GenerateImuData(int const num_samples, uint64_t const timespan_ns) {
    spline::Se3Spline trajectory{TimedSphereTrajectorySpline(5 * num_samples, timespan_ns)};
    std::set<uint64_t> const times{SampleTimes(num_samples, timespan_ns)};

    // TODO KEEP THIS?
    for (int i{0}; i < trajectory.Size(); ++i) {
        trajectory.MutableControlPoints().col(i) =
            geometry::Log(geometry::Exp(trajectory.MutableControlPoints().col(i)).inverse());
    }

    ImuMeasurements imu_data;
    for (auto const time_i : times) {
        auto const velocity_t{trajectory.Evaluate(time_i, spline::DerivativeOrder::First)};
        auto const acceleration_t{trajectory.Evaluate(time_i, spline::DerivativeOrder::Second)};

        if (not(velocity_t.has_value() and acceleration_t.has_value())) {
            throw std::runtime_error("GenerateImuData() failed trajectory.Evaluate().");  // LCOV_EXCL_LINE
        }

        imu_data.insert({time_i, {velocity_t->head<3>(), acceleration_t->tail<3>()}});
    }

    return {imu_data, trajectory};
}

}  // namespace reprojection::testing_mocks