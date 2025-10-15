#include "dlt.hpp"

#include <gtest/gtest.h>

#include "multiple_view_geometry_data_generator.hpp"

// TODO(Jack): I think we could add a test where we check more properties, like for example PC=0, etc. Even though these
// might already be checked in some sub-tests for specific test data, we should be able to do it for all executions.

using namespace reprojection_calibration::pnp;

TEST(Dlt, TestDlt) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    for (size_t i{0}; i < 20; ++i) {
        MvgFrame const frame_i{generator.Generate()};
        auto const [tf, K]{Dlt(frame_i.pixels, frame_i.points)};

        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant

        Se3 const pose_i{ToSe3(tf)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose)) << "Result:\n" << pose_i << "\nexpected result:\n" << frame_i.pose;

        EXPECT_TRUE(K.isUpperTriangular());  // Property of camera intrinsic matrix
        EXPECT_TRUE(K.isApprox(generator.GetK()));
    }
}
