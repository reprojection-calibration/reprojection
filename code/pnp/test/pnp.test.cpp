#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"

using namespace reprojection;

// TODO(Jack): Test all functions with noisy data!

TEST(Pnp, TestPnp) {
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera({600, 600, 360, 240})),
        false)};

    std::vector<Frame> const frames{generator.GenerateBatchFrames(20)};
    for (auto const& frame : frames) {
        pnp::PnpResult const pnp_result{pnp::Pnp(frame.bundle)};
        EXPECT_TRUE(std::holds_alternative<Isometry3d>(pnp_result));

        Isometry3d const pose_i{std::get<Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(frame.pose));
    }
}

TEST(Pnp, TestPnpFlat) {
    Array4d const intrinsics{1, 1, 0, 0};  // Equivalent to K = I_3x3 Pixels must be in normalized image space for Dlt22
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(
        std::unique_ptr<projection_functions::Camera>(new projection_functions::PinholeCamera(intrinsics)),
        true)};  // Points must have Z=0 (flat = true)

    std::vector<Frame> const frames{generator.GenerateBatchFrames(20)};
    for (auto const& frame : frames) {
        pnp::PnpResult const pnp_result{pnp::Pnp(frame.bundle)};
        EXPECT_TRUE(std::holds_alternative<Isometry3d>(pnp_result));

        Isometry3d const pose_i{std::get<Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(frame.pose));
    }
}

TEST(Pnp, TestMismatchedCorrespondence) {
    // TODO(Jack): Should the Bundle type not even allow a construction from mismatched sizes?
    MatrixX2d const four_pixels(4, 2);
    MatrixX3d const five_points(5, 3);
    pnp::PnpResult const pnp_result{pnp::Pnp({four_pixels, five_points})};

    EXPECT_TRUE(std::holds_alternative<pnp::PnpStatusCode>(pnp_result));
    pnp::PnpStatusCode const status{std::get<pnp::PnpStatusCode>(pnp_result)};
    EXPECT_EQ(status, pnp::PnpStatusCode::MismatchedCorrespondences);
}

TEST(Pnp, TestNotEnoughPoints) {
    MatrixX2d const five_pixels(5, 2);
    MatrixX3d const five_points(5, 3);
    pnp::PnpResult const pnp_result{pnp::Pnp({five_pixels, five_points})};

    EXPECT_TRUE(std::holds_alternative<pnp::PnpStatusCode>(pnp_result));
    pnp::PnpStatusCode const status{std::get<pnp::PnpStatusCode>(pnp_result)};
    EXPECT_EQ(status, pnp::PnpStatusCode::NotEnoughPoints);
}
