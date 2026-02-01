#include "testing_mocks/imu_data_generator.hpp"

#include "spline/se3_spline.hpp"

#include "constants.hpp"
#include "sphere_trajectory.hpp"

namespace reprojection::testing_mocks {

ImuData GenerateImuData(int const num_measurements) {
    // TODO(Jack): Is two times num_samples dense enough to prevent sampling artifacts?
    int const num_control_points{2 * num_measurements};
    spline::Se3Spline se3_spline{constants::t0_ns, constants::delta_t_ns};
    for (auto const& pose : SphereTrajectory(num_control_points, constants::trajectory)) {
        se3_spline.AddControlPoint(pose);
    }

    ImuData data;
    for (int i{0}; i < num_measurements; ++i) {
        double const elapsed_trajectory{static_cast<double>(i) / num_measurements};
        uint64_t const spline_time{constants::t0_ns +
                                   static_cast<uint64_t>((num_control_points - spline::constants::degree) *
                                                         constants::delta_t_ns * elapsed_trajectory)};

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