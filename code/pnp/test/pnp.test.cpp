#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "testing_mocks/mvg_generator.hpp"

using namespace reprojection;

// TODO(Jack): Test all functions with noisy data!

TEST(Pnp, TestPnp) {
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera({600, 600, 360, 240})),
        false)};

    std::vector<Frame> const frames{generator.GenerateBatch(20)};
    for (auto const& frame : frames) {
        pnp::PnpResult const pnp_result{pnp::Pnp(frame.bundle)};
        EXPECT_TRUE(std::holds_alternative<pnp::PnpOutput>(pnp_result));

        auto const output_i{std::get<pnp::PnpOutput>(pnp_result)};
        EXPECT_TRUE(output_i.pose.isApprox(frame.pose));
        EXPECT_LT(output_i.reprojection_error, 1e-9);  // Because there is no noise the error is extremely low!
    }
}

TEST(Pnp, TestPnpFlat) {
    Array4d const intrinsics{1, 1, 0, 0};  // Equivalent to K = I_3x3 Pixels must be in normalized image space for Dlt22
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)),
        true)};  // Points must have Z=0 (flat = true)

    std::vector<Frame> const frames{generator.GenerateBatch(20)};
    for (auto const& frame : frames) {
        pnp::PnpResult const pnp_result{pnp::Pnp(frame.bundle)};
        EXPECT_TRUE(std::holds_alternative<pnp::PnpOutput>(pnp_result));

        auto const output_i{std::get<pnp::PnpOutput>(pnp_result)};
        EXPECT_TRUE(output_i.pose.isApprox(frame.pose));
        EXPECT_LT(output_i.reprojection_error, 1e-9);
    }
}

TEST(Pnp, TestNotEnoughPoints) {
    MatrixX2d const five_pixels(5, 2);
    MatrixX3d const five_points(5, 3);
    pnp::PnpResult const pnp_result{pnp::Pnp({five_pixels, five_points})};

    EXPECT_TRUE(std::holds_alternative<pnp::PnpStatusCode>(pnp_result));
    pnp::PnpStatusCode const status{std::get<pnp::PnpStatusCode>(pnp_result)};
    EXPECT_EQ(status, pnp::PnpStatusCode::NotEnoughPoints);
}
