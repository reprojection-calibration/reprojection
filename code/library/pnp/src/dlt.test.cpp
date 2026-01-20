#include "dlt.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"

// TODO(Jack): I think we could add a test where we check more properties, like for example PC=0, etc. Even though these
// might already be checked in some sub-tests for specific test data, we should be able to do it for all executions.

using namespace reprojection;

TEST(PnpDlt, TestDlt23) {
    testing_mocks::MvgGenerator const generator{
        testing_mocks::MvgGenerator(CameraModel::Pinhole, Array4d{600, 600, 360, 240}, {0, 720, 0, 480}, false)};
    CameraCalibrationData const data{generator.GenerateBatch(20)};

    for (auto const& [timestamp_ns, frame_i] : data.frames) {
        auto const [tf, K]{pnp::Dlt23(frame_i.extracted_target.bundle)};

        // WARN(Jack): Unprotected optional access! Do we need a better strategy here? The mvg test data should
        // definitely have filled out this value!
        Isometry3d const gt_pose_i{geometry::Exp(data.frames.at(timestamp_ns).initial_pose.value())};

        EXPECT_TRUE(tf.isApprox(gt_pose_i)) << "Result:\n"
                                            << tf.matrix() << "\nexpected result:\n"
                                            << gt_pose_i.matrix();
        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant
        EXPECT_TRUE(K.isApprox(data.initial_intrinsics));
    }
}

TEST(PnpDlt, TestDlt22) {
    // Points must have Z=0 (flat = true) for Dlt22
    testing_mocks::MvgGenerator const generator{
        testing_mocks::MvgGenerator(CameraModel::Pinhole, Array4d{1, 1, 0, 0}, {-1, 1, -1, 1}, true)};
    CameraCalibrationData const data{generator.GenerateBatch(20)};

    for (auto const& [timestamp_ns, frame_i] : data.frames) {
        auto const tf{pnp::Dlt22(frame_i.extracted_target.bundle)};

        // WARN(Jack): Unprotected optional access! Do we need a better strategy here? The mvg test data should
        // definitely have filled out this value!
        Isometry3d const gt_pose_i{geometry::Exp(data.frames.at(timestamp_ns).initial_pose.value())};

        EXPECT_TRUE(tf.isApprox(gt_pose_i)) << "Result:\n"
                                            << tf.matrix() << "\nexpected result:\n"
                                            << gt_pose_i.matrix();
        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant
    }
}
