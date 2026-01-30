
#include <gtest/gtest.h>

#include "spline/se3_spline.hpp"
#include "types/eigen_types.hpp"

#include "constants.hpp"
#include "sphere_trajectory.hpp"

namespace reprojection::testing_mocks {

// TODO(Jack): Make sure this is not already defined somewhere else, I think it is!
struct ImuMeasurement {
    Vector3d angular_velocity;
    Vector3d linear_acceleration;
};

using ImuData = std::map<uint64_t, ImuMeasurement>;

ImuData GenerateImuData(int const num_measurements) {
    // TODO is two times num_samples dense enough to prevent sampling artifacts?
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
            throw std::runtime_error("GenerateImuData() failed se3_spline.Evaluate().");
        }

        data[spline_time] = {velocity_t->bottomRows(3), acceleration_t->topRows(3)};
    }

    return data;
}

}  // namespace reprojection::testing_mocks

using namespace reprojection;

TEST(TestingMocksImuDataGenerator, TestGenerateImuData) {
    testing_mocks::ImuData const data{testing_mocks::GenerateImuData(100)};

    uint64_t gt_timestamp_ns{0};
    for (auto const& [timestamp_ns, _] : data) {
        // NOTE(Jack): We allow a one nanosecond tolerance to account for rounding/casting errors.
        EXPECT_NEAR(timestamp_ns, gt_timestamp_ns, 1);

        gt_timestamp_ns += 1970000;
    }
    EXPECT_NEAR(gt_timestamp_ns, 195030000 + 1970000, 1);
}