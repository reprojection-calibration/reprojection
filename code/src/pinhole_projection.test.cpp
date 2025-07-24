#include "reprojection/pinhole_projection.hpp"

#include <gtest/gtest.h>

#include <array>

using namespace reprojection_calibration::reprojection;

TEST(TestPinholeProjection, FiveSimplePoints) {
    std::array<double, 4> const pinhole_intrinsics{100, 100, 250, 250};

    std::array<double, 3> const point_1{1, -1, 10};
    auto const [u_1, v_1]{PinholeProjection(pinhole_intrinsics.data(), point_1.data())};
    EXPECT_FLOAT_EQ(u_1, 260);
    EXPECT_FLOAT_EQ(v_1, 240);

    std::array<double, 3> const point_2{-1, 1, 10};
    auto const [u_2, v_2]{PinholeProjection(pinhole_intrinsics.data(), point_2.data())};
    EXPECT_FLOAT_EQ(u_2, 240);
    EXPECT_FLOAT_EQ(v_2, 260);

    std::array<double, 3> const point_3{-1, -1, 10};
    auto const [u_3, v_3]{PinholeProjection(pinhole_intrinsics.data(), point_3.data())};
    EXPECT_FLOAT_EQ(u_3, 240);
    EXPECT_FLOAT_EQ(v_3, 240);

    std::array<double, 3> const point_4{1, 1, 10};
    auto const [u_4, v_4]{PinholeProjection(pinhole_intrinsics.data(), point_4.data())};
    EXPECT_FLOAT_EQ(u_4, 260);
    EXPECT_FLOAT_EQ(v_4, 260);

    std::array<double, 3> const point_5{0, 0, 10};
    auto const [u_5, v_5]{PinholeProjection(pinhole_intrinsics.data(), point_5.data())};
    EXPECT_FLOAT_EQ(u_5, 250);
    EXPECT_FLOAT_EQ(v_5, 250);
}
