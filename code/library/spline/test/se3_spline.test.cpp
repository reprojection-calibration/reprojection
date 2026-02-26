#include "spline/se3_spline.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;
using namespace spline;
using enum DerivativeOrder;

// Read the comments in TEST(Spline_r3Spline, TestEvaluateValidity) for more context.
TEST(SplineSe3Spline, TestEvaluateValidity) {
    Se3Spline const empty_spline{{}, {100, 5}};
    EXPECT_FALSE(empty_spline.Evaluate(100, Null));

    Matrix2NXd const four_control_points{Matrix2NXd::Zero(2 * constants::states, constants::order)};
    Se3Spline const one_segment_spline{four_control_points, {100, 5}};

    EXPECT_TRUE(one_segment_spline.Evaluate(100, Null));
    EXPECT_FALSE(one_segment_spline.Evaluate(105, Null));

    Matrix2NXd const five_control_points{Matrix2NXd::Zero(2 * constants::states, constants::order + 1)};
    Se3Spline const two_segment_spline{five_control_points, {100, 5}};
    EXPECT_TRUE(two_segment_spline.Evaluate(105, Null));
}

TEST(SplineSe3Spline, TestEvaluate) {
    std::vector<Vector6d> se3_control_points;
    Vector6d control_point_i{Vector6d::Zero()};
    se3_control_points.push_back(control_point_i);

    for (int i{1}; i < constants::order; ++i) {
        Vector6d delta;
        delta.topRows(3) = (static_cast<double>(i) / 10) * Vector3d::Ones();
        delta.bottomRows(3) = i * Vector3d::Ones();

        control_point_i = delta + control_point_i;

        se3_control_points.push_back(control_point_i);
    }

    uint64_t const delta_t_ns{5};
    Se3Spline const spline{se3_control_points, {100, delta_t_ns}};

    // Heuristic test as we have no theoretical testing strategy at this time.
    for (uint64_t i{0}; i < delta_t_ns; ++i) {
        auto const p_i{spline.Evaluate(100 + i, Null)};
        ASSERT_TRUE(p_i.has_value());
    }

    auto const p_0{spline.Evaluate(100, Null)};
    EXPECT_FLOAT_EQ(geometry::Exp(p_0.value()).matrix().diagonal().sum(),
                    3.9593055);  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
    // that we can detect changes to the implementation quickly (hopefully. )
}