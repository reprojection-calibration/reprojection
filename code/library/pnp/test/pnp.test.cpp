#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

// TODO(Jack): Test all functions with noisy data!

TEST(Pnp, TestPnp) {
    CameraCalibrationData const gt_data{testing_mocks::GenerateMvgData(
        20, 1e9, CameraModel::Pinhole, testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds, false)};

    for (auto const& [_, frame_i] : gt_data.frames) {
        pnp::PnpResult const pnp_result{pnp::Pnp(frame_i.extracted_target.bundle, testing_utilities::image_bounds)};
        EXPECT_TRUE(std::holds_alternative<Isometry3d>(pnp_result));

        Isometry3d const gt_tf_co_w{geometry::Exp(frame_i.initial_pose.value())};

        Isometry3d const tf_co_w{std::get<Isometry3d>(pnp_result)};
        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w)) << "Result:\n"
                                                  << tf_co_w.matrix() << "\nexpected result:\n"
                                                  << gt_tf_co_w.matrix();
    }
}

TEST(Pnp, TestPnpFlat) {
    CameraCalibrationData const data{testing_mocks::GenerateMvgData(20, 1e9, CameraModel::Pinhole,
                                                                    testing_utilities::unit_pinhole_intrinsics,
                                                                    testing_utilities::unit_image_bounds, true)};

    for (auto const& [_, frame_i] : data.frames) {
        pnp::PnpResult const pnp_result{pnp::Pnp(frame_i.extracted_target.bundle)};
        EXPECT_TRUE(std::holds_alternative<Isometry3d>(pnp_result));

        Isometry3d const gt_tf_co_w{geometry::Exp(frame_i.initial_pose.value())};

        Isometry3d const tf_co_w{std::get<Isometry3d>(pnp_result)};
        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w)) << "Result:\n"
                                                  << tf_co_w.matrix() << "\nexpected result:\n"
                                                  << gt_tf_co_w.matrix();
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
    pnp::PnpResult const pnp_result{pnp::Pnp({pixels, points}, testing_utilities::unit_image_bounds)};

    ASSERT_TRUE(std::holds_alternative<pnp::PnpErrorCode>(pnp_result));
    pnp::PnpErrorCode const error_code{std::get<pnp::PnpErrorCode>(pnp_result)};
    EXPECT_EQ(error_code, pnp::PnpErrorCode::ContainsNan);
}
