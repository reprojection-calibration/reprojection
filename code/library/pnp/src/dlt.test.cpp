#include "dlt.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"

// TODO(Jack): I think we could add a test where we check more properties, like for example PC=0, etc. Even though these
// might already be checked in some sub-tests for specific test data, we should be able to do it for all executions.

using namespace reprojection;

TEST(PnpDlt, TestDlt23) {
    CameraCalibrationData const gt_data{testing_mocks::GenerateMvgData(
        20, 1e9, CameraModel::Pinhole, testing_utilities::pinhole_intrinsics, testing_utilities::image_bounds, false)};

    for (auto const& [timestamp_ns, frame_i] : gt_data.frames) {
        auto const [tf_co_w, K]{pnp::Dlt23(frame_i.extracted_target.bundle)};

        // WARN(Jack): Unprotected optional access! Do we need a better strategy here? The mvg test data should
        // definitely have filled out this value!
        Isometry3d const gt_tf_w_co{geometry::Exp(gt_data.frames.at(timestamp_ns).initial_pose.value())};
        Isometry3d const gt_tf_co_w{gt_tf_w_co.inverse()};

        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w)) << "Result:\n"
                                                  << tf_co_w.matrix() << "\nexpected result:\n"
                                                  << gt_tf_co_w.matrix();
        EXPECT_FLOAT_EQ(tf_co_w.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant
        EXPECT_TRUE(K.isApprox(gt_data.initial_intrinsics));
    }
}

TEST(PnpDlt, TestDlt22) {
    // Points must have Z=0 (flat = true) for Dlt22
    CameraCalibrationData const gt_data{testing_mocks::GenerateMvgData(20, 1e9, CameraModel::Pinhole,
                                                                       testing_utilities::unit_pinhole_intrinsics,
                                                                       testing_utilities::unit_image_bounds, true)};

    for (auto const& [timestamp_ns, frame_i] : gt_data.frames) {
        auto const tf_co_w{pnp::Dlt22(frame_i.extracted_target.bundle)};

        // WARN(Jack): Unprotected optional access! Do we need a better strategy here? The mvg test data should
        // definitely have filled out this value!
        Isometry3d const gt_tf_w_co{geometry::Exp(gt_data.frames.at(timestamp_ns).initial_pose.value())};
        Isometry3d const gt_tf_co_w{gt_tf_w_co.inverse()};

        EXPECT_TRUE(tf_co_w.isApprox(gt_tf_co_w)) << "Result:\n"
                                                  << tf_co_w.matrix() << "\nexpected result:\n"
                                                  << gt_tf_co_w.matrix();
        EXPECT_FLOAT_EQ(tf_co_w.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant
    }
}
