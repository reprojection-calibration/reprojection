#include "pnp/pnp.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

// TODO(Jack): Test all functions with noisy data!

TEST(Pnp, TestPnp) {
    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    auto const [targets, gt_frames]{
        testing_mocks::GenerateMvgData(sensor, CameraState{testing_utilities::pinhole_intrinsics}, 60, 1, false)};

    for (auto const& [timestamp_ns, target_i] : targets) {
        pnp::PnpResult const pnp_result{pnp::Pnp(target_i.bundle, sensor.bounds)};
        ASSERT_TRUE(std::holds_alternative<pnp::PoseWithCost>(pnp_result));

        pnp::PoseWithCost const result{std::get<pnp::PoseWithCost>(pnp_result)};
        auto const [tf_co_w, cost]{result};

        Isometry3d const gt_tf_co_w{geometry::Exp(gt_frames.at(timestamp_ns).pose)};
        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w)) << "Result:\n"
                                                  << tf_co_w.matrix() << "\nexpected result:\n"
                                                  << gt_tf_co_w.matrix();
        EXPECT_NEAR(cost, 0.0, 1e-15);
    }
}

TEST(Pnp, TestPnpFlat) {
    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::unit_image_bounds};
    auto const [targets, gt_frames]{
        testing_mocks::GenerateMvgData(sensor, CameraState{testing_utilities::unit_pinhole_intrinsics}, 60, 1, true)};

    for (auto const& [timestamp_ns, target_i] : targets) {
        pnp::PnpResult const pnp_result{pnp::Pnp(target_i.bundle)};
        ASSERT_TRUE(std::holds_alternative<pnp::PoseWithCost>(pnp_result));

        pnp::PoseWithCost const result{std::get<pnp::PoseWithCost>(pnp_result)};
        auto const [tf_co_w, cost]{result};

        Isometry3d const gt_tf_co_w{geometry::Exp(gt_frames.at(timestamp_ns).pose)};
        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w)) << "Result:\n"
                                                  << tf_co_w.matrix() << "\nexpected result:\n"
                                                  << gt_tf_co_w.matrix();
        EXPECT_NEAR(cost, 0.0, 1e-15);
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
    Bundle const bundle{MatrixX2d::Zero(10, 2), MatrixX3d::Zero(10, 3)};
    pnp::PnpResult const pnp_result{pnp::Pnp(bundle)};

    ASSERT_TRUE(std::holds_alternative<pnp::PnpErrorCode>(pnp_result));
    pnp::PnpErrorCode const error_code{std::get<pnp::PnpErrorCode>(pnp_result)};
    EXPECT_EQ(error_code, pnp::PnpErrorCode::InvalidDlt);
}

TEST(Pnp, TestFailedDlt) {
    Bundle const bundle{MatrixX2d::Zero(10, 2), MatrixX3d::Zero(10, 3)};
    pnp::PnpResult const pnp_result{pnp::Pnp(bundle, testing_utilities::unit_image_bounds)};

    ASSERT_TRUE(std::holds_alternative<pnp::PnpErrorCode>(pnp_result));
    pnp::PnpErrorCode const error_code{std::get<pnp::PnpErrorCode>(pnp_result)};
    EXPECT_EQ(error_code, pnp::PnpErrorCode::FailedDlt);
}
