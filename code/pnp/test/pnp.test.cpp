#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"

using namespace reprojection;
using namespace reprojection::pnp;

// TODO(Jack): Test all functions with noisy data!

TEST(Pnp, TestPnp) {
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(false)};
    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};

        PnpResult const pnp_result{Pnp(frame_i.pixels, frame_i.points)};
        EXPECT_TRUE(std::holds_alternative<Eigen::Isometry3d>(pnp_result));

        Eigen::Isometry3d const pose_i{std::get<Eigen::Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose));
    }
}

TEST(Pnp, TestPnpFlat) {
    Eigen::Matrix3d const K{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // Pixels must be in normalized space for Dlt22
    testing_mocks::MvgGenerator const generator{
        testing_mocks::MvgGenerator(true, K)};  // Points must have Z=0 (flat = true)

    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};

        PnpResult const pnp_result{Pnp(frame_i.pixels, frame_i.points)};
        EXPECT_TRUE(std::holds_alternative<Eigen::Isometry3d>(pnp_result));

        Eigen::Isometry3d const pose_i{std::get<Eigen::Isometry3d>(pnp_result)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose));
    }
}

TEST(Pnp, TestMismatchedCorrespondence) {
    Eigen::MatrixX2d const four_pixels(4, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(four_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(pnp_status_code, PnpStatusCode::MismatchedCorrespondences);
}

TEST(Pnp, TestNotEnoughPoints) {
    Eigen::MatrixX2d const five_pixels(5, 2);
    Eigen::MatrixX3d const five_points(5, 3);
    PnpResult const pnp_result{Pnp(five_pixels, five_points)};

    EXPECT_TRUE(std::holds_alternative<PnpStatusCode>(pnp_result));
    PnpStatusCode const pnp_status_code{std::get<PnpStatusCode>(pnp_result)};
    EXPECT_EQ(pnp_status_code, PnpStatusCode::NotEnoughPoints);
}
