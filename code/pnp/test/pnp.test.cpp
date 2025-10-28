#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"

using namespace reprojection;

// TODO(Jack): Test all functions with noisy data!

TEST(Pnp, TestPnp) {
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(false)};
    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};

        pnp::PnpResult const pnp_result{pnp::Pnp(frame_i.pixels, frame_i.points)};
        EXPECT_TRUE(std::holds_alternative<Isometry3d>(pnp_result));

        Isometry3d const pose_i{std::get<Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose));
    }
}

TEST(Pnp, TestPnpFlat) {
    Array4d const K{1, 1, 0, 0};  // Equivalent to K = I_3x3 Pixels must be in normalized image space for Dlt22
    testing_mocks::MvgGenerator const generator{
        testing_mocks::MvgGenerator(true, K)};  // Points must have Z=0 (flat = true)

    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};

        pnp::PnpResult const pnp_result{pnp::Pnp(frame_i.pixels, frame_i.points)};
        EXPECT_TRUE(std::holds_alternative<Isometry3d>(pnp_result));

        Isometry3d const pose_i{std::get<Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose));
    }
}

TEST(Pnp, TestMismatchedCorrespondence) {
    MatrixX2d const four_pixels(4, 2);
    MatrixX3d const five_points(5, 3);
    pnp::PnpResult const pnp_result{pnp::Pnp(four_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<pnp::PnpStatusCode>(pnp_result));
    pnp::PnpStatusCode const status{std::get<pnp::PnpStatusCode>(pnp_result)};
    EXPECT_EQ(status, pnp::PnpStatusCode::MismatchedCorrespondences);
}

TEST(Pnp, TestNotEnoughPoints) {
    MatrixX2d const five_pixels(5, 2);
    MatrixX3d const five_points(5, 3);
    pnp::PnpResult const pnp_result{pnp::Pnp(five_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<pnp::PnpStatusCode>(pnp_result));
    pnp::PnpStatusCode const status{std::get<pnp::PnpStatusCode>(pnp_result)};
    EXPECT_EQ(status, pnp::PnpStatusCode::NotEnoughPoints);
}
