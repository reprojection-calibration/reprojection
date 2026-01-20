#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"

using namespace reprojection;

// TODO(Jack): Test all functions with noisy data!

TEST(Pnp, TestPnp) {
    ImageBounds const bounds{0, 720, 0, 480};
    testing_mocks::MvgGenerator const generator{CameraModel::Pinhole, Array4d{600, 600, 360, 240}, bounds, false};
    CameraCalibrationData const data{generator.GenerateBatch(20)};

    for (auto const& [_, frame_i] : data.frames) {
        pnp::PnpResult const pnp_result{pnp::Pnp(frame_i.extracted_target.bundle, bounds)};
        EXPECT_TRUE(std::holds_alternative<Isometry3d>(pnp_result));

        // WARN(Jack): Unprotected optional access! Do we need a better strategy here? The mvg test data should
        // definitely have filled out this value!
        Array6d const pose_i{geometry::Log(std::get<Isometry3d>(pnp_result))};
        EXPECT_TRUE(pose_i.isApprox(frame_i.initial_pose.value()));
    }
}

TEST(Pnp, TestPnpFlat) {
    ImageBounds const bounds{-1, 1, -1, 1};
    testing_mocks::MvgGenerator const generator{CameraModel::Pinhole, Array4d{1, 1, 0, 0}, bounds, true};
    CameraCalibrationData const data{generator.GenerateBatch(20)};

    for (auto const& [_, frame_i] : data.frames) {
        pnp::PnpResult const pnp_result{pnp::Pnp(frame_i.extracted_target.bundle, bounds)};
        EXPECT_TRUE(std::holds_alternative<Isometry3d>(pnp_result));

        // WARN(Jack): Unprotected optional access! Do we need a better strategy here? The mvg test data should
        // definitely have filled out this value!
        Array6d const pose_i{geometry::Log(std::get<Isometry3d>(pnp_result))};
        EXPECT_TRUE(pose_i.isApprox(frame_i.initial_pose.value()));
    }
}

TEST(Pnp, TestNotEnoughPoints) {
    MatrixX2d const five_pixels(5, 2);
    MatrixX3d const five_points(5, 3);
    pnp::PnpResult const pnp_result{pnp::Pnp({five_pixels, five_points})};

    ASSERT_TRUE(std::holds_alternative<pnp::PnpErrorCode>(pnp_result));
    pnp::PnpErrorCode const error_code{std::get<pnp::PnpErrorCode>(pnp_result)};
    EXPECT_EQ(error_code, pnp::PnpErrorCode::InvalidDlt);
}

TEST(Pnp, TestForgetToPassBounds) {
    MatrixX2d const pixels(10, 2);
    MatrixX3d const points(10, 3);
    pnp::PnpResult const pnp_result{pnp::Pnp({pixels, points})};

    ASSERT_TRUE(std::holds_alternative<pnp::PnpErrorCode>(pnp_result));
    pnp::PnpErrorCode const error_code{std::get<pnp::PnpErrorCode>(pnp_result)};
    EXPECT_EQ(error_code, pnp::PnpErrorCode::InvalidDlt);
}

// WARN(Jack): I am not 100% sure this will always result in nan, but if you throw in total junk data it seems like to
// DLT evaluation totally collapses.
TEST(Pnp, TestNans) {
    MatrixX2d const pixels(10, 2);
    MatrixX3d const points(10, 3);
    pnp::PnpResult const pnp_result{pnp::Pnp({pixels, points}, ImageBounds{-1, 1, -1, 1})};

    ASSERT_TRUE(std::holds_alternative<pnp::PnpErrorCode>(pnp_result));
    pnp::PnpErrorCode const error_code{std::get<pnp::PnpErrorCode>(pnp_result)};
    EXPECT_EQ(error_code, pnp::PnpErrorCode::ContainsNan);
}
