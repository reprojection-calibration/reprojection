#include "reprojection/unified_camera_model_projection.hpp"

#include <gtest/gtest.h>

#include <array>

using namespace reprojection_calibration::reprojection;

TEST(TestUcmProjection, FiveSimplePoints) {
    std::array<double, 5> const ucm_intrinsics{100, 100, 250, 250, 0.5};

    std::array<double, 3> const point_1{1, -1, 10};
    auto const [u_1, v_1]{UcmProjection(ucm_intrinsics.data(), point_1.data())};
    EXPECT_FLOAT_EQ(u_1, 259.9505);
    EXPECT_FLOAT_EQ(v_1, 240.0495);

    std::array<double, 3> const point_2{-1, 1, 10};
    auto const [u_2, v_2]{UcmProjection(ucm_intrinsics.data(), point_2.data())};
    EXPECT_FLOAT_EQ(u_2, 240.0495);
    EXPECT_FLOAT_EQ(v_2, 259.9505);

    std::array<double, 3> const point_3{-1, -1, 10};
    auto const [u_3, v_3]{UcmProjection(ucm_intrinsics.data(), point_3.data())};
    EXPECT_FLOAT_EQ(u_3, 240.0495);
    EXPECT_FLOAT_EQ(v_3, 240.0495);

    std::array<double, 3> const point_4{1, 1, 10};
    auto const [u_4, v_4]{UcmProjection(ucm_intrinsics.data(), point_4.data())};
    EXPECT_FLOAT_EQ(u_4, 259.9505);
    EXPECT_FLOAT_EQ(v_4, 259.9505);

    std::array<double, 3> const point_5{0, 0, 10};
    auto const [u_5, v_5]{UcmProjection(ucm_intrinsics.data(), point_5.data())};
    EXPECT_FLOAT_EQ(u_5, 250);
    EXPECT_FLOAT_EQ(v_5, 250);
}
