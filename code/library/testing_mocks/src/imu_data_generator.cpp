#include "testing_mocks/imu_data_generator.hpp"

#include "spline/se3_spline.hpp"

#include "constants.hpp"
#include "sphere_trajectory.hpp"

namespace reprojection::testing_mocks {

ImuData GenerateImuData(int const num_measurements, uint64_t const timespan_ns) {
    // TODO(Jack): Is two times num_samples dense enough to prevent sampling artifacts?
    int const num_control_points{2 * num_measurements};
    uint64_t const delta_t_ns{timespan_ns / num_control_points};
    spline::Se3Spline se3_spline{0, delta_t_ns};
    for (auto const& pose : SphereTrajectory(num_control_points, constants::trajectory)) {
        se3_spline.AddControlPoint(pose);
    }

    ImuData data;
    uint64_t const pose_delta_t_ns{timespan_ns / num_measurements};
    for (int i{0}; i < num_measurements; ++i) {
        double const elapsed_trajectory{static_cast<double>(i) / num_measurements};
        uint64_t const spline_time{static_cast<uint64_t>((num_measurements - spline::constants::degree) *
                                                         pose_delta_t_ns * elapsed_trajectory)};

        auto const velocity_t{se3_spline.Evaluate(spline_time, spline::DerivativeOrder::First)};
        auto const acceleration_t{se3_spline.Evaluate(spline_time, spline::DerivativeOrder::Second)};
        if (not(velocity_t.has_value() and acceleration_t.has_value())) {
            throw std::runtime_error("GenerateImuData() failed se3_spline.Evaluate().");  // LCOV_EXCL_LINE
        }

        data[spline_time] = {velocity_t->topRows(3), acceleration_t->bottomRows(3)};
    }

    return data;
}

}  // namespace reprojection::testing_mocks