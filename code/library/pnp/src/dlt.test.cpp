#include "dlt.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

// TODO(Jack): I think we could add a test where we check more properties, like for example PC=0, etc. Even though these
// might already be checked in some sub-tests for specific test data, we should be able to do it for all executions.

using namespace reprojection;

TEST(PnpDlt, TestDlt23) {
    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState const intrinsics{testing_utilities::pinhole_intrinsics};
    auto const [targets, gt_frames]{testing_mocks::GenerateMvgData(sensor, intrinsics, 60, 1, false)};

    for (auto const& [timestamp_ns, target_i] : targets) {
        auto const dlt_result{pnp::Dlt23(target_i.bundle)};
        ASSERT_TRUE(dlt_result.has_value());
        auto const [tf_co_w, K_vec]{*dlt_result};

        Isometry3d const gt_tf_co_w{geometry::Exp(gt_frames.at(timestamp_ns).pose)};

        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w)) << "Result:\n"
                                                  << tf_co_w.matrix() << "\nexpected result:\n"
                                                  << gt_tf_co_w.matrix();
        EXPECT_FLOAT_EQ(tf_co_w.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant
        EXPECT_TRUE(K_vec.isApprox(intrinsics.intrinsics)) << "Result:\n"
                                                           << K_vec.transpose() << "\nexpected result:\n"
                                                           << intrinsics.intrinsics.transpose();
    }
}

TEST(PnpDlt, TestDlt23FailedDlt) {
    // An empty bundle will cause the svd inside the DLT to fail.
    Bundle const bundle{MatrixX2d(10, 2), MatrixX3d(10, 3)};

    auto const dlt_result{pnp::Dlt23(bundle)};
    EXPECT_FALSE(dlt_result.has_value());
}

TEST(PnpDlt, TestDlt22) {
    // Points must have Z=0 (flat = true) for Dlt22
    CameraInfo const sensor{"", CameraModel::Pinhole, testing_utilities::unit_image_bounds};
    auto const [targets, gt_frames]{
        testing_mocks::GenerateMvgData(sensor, CameraState{testing_utilities::unit_pinhole_intrinsics}, 60, 1, true)};

    for (auto const& [timestamp_ns, target_i] : targets) {
        auto const tf_co_w{pnp::Dlt22(target_i.bundle)};
        ASSERT_TRUE(tf_co_w.has_value());

        Isometry3d const gt_tf_co_w{geometry::Exp(gt_frames.at(timestamp_ns).pose)};

        EXPECT_TRUE(tf_co_w->isApprox(gt_tf_co_w)) << "Result:\n"
                                                   << tf_co_w->matrix() << "\nexpected result:\n"
                                                   << gt_tf_co_w.matrix();
        EXPECT_FLOAT_EQ(tf_co_w->linear().determinant(), 1);  // Property of rotation matrix - positive one determinant
    }
}

TEST(PnpDlt, TestDlt22FailedDlt) {
    // An empty bundle will cause the svd inside the DLT to fail.
    Bundle const bundle{MatrixX2d(10, 2), MatrixX3d(10, 3)};

    auto const tf_co_w{pnp::Dlt22(bundle)};
    EXPECT_FALSE(tf_co_w.has_value());
}
