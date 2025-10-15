#include "se3_spline.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "utilities_testing.hpp"

using namespace reprojection;
using namespace reprojection::spline;

TEST(Se3Spline, TestSe3SplineInvalidEvaluateConditions) {
    Se3Spline se3_spline{100, 5};
    EXPECT_EQ(se3_spline.Evaluate(115), std::nullopt);

    for (int i{0}; i < constants::k; ++i) {
        se3_spline.AddKnot(Eigen::Isometry3d::Identity());
    }

    EXPECT_NE(se3_spline.Evaluate(100), std::nullopt);
    EXPECT_EQ(se3_spline.Evaluate(105), std::nullopt);

    se3_spline.AddKnot(Eigen::Isometry3d::Identity());
    EXPECT_NE(se3_spline.Evaluate(105), std::nullopt);
}

TEST(Se3Spline, TestSe3SplineEvaluate) {
    uint64_t const delta_t_ns{5};
    Se3Spline se3_spline{100, delta_t_ns};

    Eigen::Isometry3d knot_i{Eigen::Isometry3d::Identity()};
    se3_spline.AddKnot(knot_i);

    for (int i{1}; i < constants::k; ++i) {
        Eigen::Isometry3d delta{Eigen::Isometry3d::Identity()};
        delta.rotate(geometry::Exp((static_cast<double>(i) / 10) * Eigen::Vector3d::Ones()));
        delta.translation() = i * VectorD::Ones();  // Using defined constant types

        knot_i = delta * knot_i;

        se3_spline.AddKnot(knot_i);
    }

    // Heuristic test as we have no theoretical testing strategy at this time.
    for (int i{0}; i < static_cast<int>(delta_t_ns); ++i) {
        auto const p_i{se3_spline.Evaluate(100 + i)};
        ASSERT_TRUE(p_i.has_value());
        EXPECT_TRUE(IsRotation(p_i->rotation().matrix()));
    }

    auto const p_0{se3_spline.Evaluate(100)};
    EXPECT_FLOAT_EQ(p_0->matrix().diagonal().sum(),
                    3.9593055);  // HEURISTIC! No theoretical testing strategy at this time - we have this here just so
    // that we can detect changes to the implementation quickly (hopefully. )
}