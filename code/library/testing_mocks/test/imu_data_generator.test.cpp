
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

ImuData GenerateImuData(int const num_samples) {
    // TODO is two times num_samples dense enough to prevent sampling artifacts?
    int const num_control_points{2 * num_samples};
    spline::Se3Spline se3_spline{constants::t0_ns, constants::delta_t_ns};
    for (auto const& pose : SphereTrajectory(num_control_points, constants::trajectory)) {
        se3_spline.AddControlPoint(pose);
    }

    ImuData data;
    for (int i{0}; i < num_samples; ++i) {
        // TODO THIS KIND OF HACKY TRAJECTORY SAMPLING IS NOW HERE AND IN THE MVG GENERATOR - DO NOT DUPLICATE THIS!
        double const elapsed_trajectory{static_cast<double>(i) / num_samples};
        assert(0 <= elapsed_trajectory and elapsed_trajectory < 1);  // TODO(Jack): Refactor to throw here.

        uint64_t const spline_time{constants::t0_ns +
                                   static_cast<uint64_t>((num_control_points - spline::constants::degree) *
                                                         constants::delta_t_ns * elapsed_trajectory)};

        auto const velocity_t{se3_spline.Evaluate(spline_time, spline::DerivativeOrder::First)};
        assert(velocity_t.has_value());  // TODO(Jack): Refactor to throw here.
        auto const acceleration_t{se3_spline.Evaluate(spline_time, spline::DerivativeOrder::Second)};
        assert(acceleration_t.has_value());  // TODO(Jack): Refactor to throw here.

        // ERROR UNPROTECTED OPTIONAL ACCESS!
        uint64_t const timestamp_ns{constants::t0_ns + constants::delta_t_ns * i};
        data[timestamp_ns] = {velocity_t->bottomRows(3), acceleration_t->topRows(3)};
    }

    return data;
}

}  // namespace reprojection::testing_mocks

using namespace reprojection;

TEST(TestingMocksImuDataGenerator, TestGenerateImuData) {
    testing_mocks::ImuData const data{testing_mocks::GenerateImuData(100)};

    uint64_t gt_timestamp_ns{0};
    for (auto const& [timestamp_ns, measurement] : data) {
        EXPECT_EQ(timestamp_ns, gt_timestamp_ns);

        gt_timestamp_ns += 1000000;
    }

    EXPECT_EQ(gt_timestamp_ns, 99000000 + 1000000);
}