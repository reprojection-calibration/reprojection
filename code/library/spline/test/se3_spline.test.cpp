#include "spline/se3_spline.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(SplineSe3Spline, TestInvalidEvaluateConditions) {
    spline::Se3Spline se3_spline{100, 5};
    EXPECT_EQ(se3_spline.Evaluate(115), std::nullopt);

    for (int i{0}; i < spline::constants::order; ++i) {
        se3_spline.AddControlPoint(Vector6d::Zero());
    }

    EXPECT_NE(se3_spline.Evaluate(100), std::nullopt);
    EXPECT_EQ(se3_spline.Evaluate(105), std::nullopt);

    se3_spline.AddControlPoint(Vector6d::Zero());
    EXPECT_NE(se3_spline.Evaluate(105), std::nullopt);
}

TEST(SplineSe3Spline, TestEvaluate) {
    uint64_t const delta_t_ns{5};
    spline::Se3Spline se3_spline{100, delta_t_ns};

    Vector6d control_point_i{Vector6d::Zero()};
    se3_spline.AddControlPoint(control_point_i);

    for (int i{1}; i < spline::constants::order; ++i) {
        Vector6d delta;
        delta.topRows(3) = (static_cast<double>(i) / 10) * Vector3d::Ones();
        delta.bottomRows(3) = i * Vector3d::Ones();

        control_point_i = delta + control_point_i;

        se3_spline.AddControlPoint(control_point_i);
    }

    // Heuristic test as we have no theoretical testing strategy at this time.
    for (int i{0}; i < static_cast<int>(delta_t_ns); ++i) {
        auto const p_i{se3_spline.Evaluate(100 + i)};
        ASSERT_TRUE(p_i.has_value());
    }

    auto const p_0{se3_spline.Evaluate(100)};
    EXPECT_FLOAT_EQ(geometry::Exp(p_0.value()).matrix().diagonal().sum(),
                    3.9593055);  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
    // that we can detect changes to the implementation quickly (hopefully. )
}