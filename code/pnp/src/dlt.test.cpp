#include "dlt.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "multiple_view_geometry_data_generator.hpp"

// TODO(Jack): I think we could add a test where we check more properties, like for example PC=0, etc. Even though these
// might already be checked in some sub-tests for specific test data, we should be able to do it for all executions.

using namespace reprojection;
using namespace reprojection::pnp;

TEST(PnpDlt, TestDlt23) {
    MvgFrameGenerator const generator{MvgFrameGenerator()};
    for (size_t i{0}; i < 20; ++i) {
        MvgFrame const frame_i{generator.Generate()};
        auto const [tf, K]{Dlt23(frame_i.pixels, frame_i.points)};

        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant

        Eigen::Vector<double, 6> const pose_i{geometry::Log(tf)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose)) << "Result:\n" << pose_i << "\nexpected result:\n" << frame_i.pose;

        EXPECT_TRUE(K.isUpperTriangular());  // Property of camera intrinsic matrix
        EXPECT_TRUE(K.isApprox(generator.GetK()));
    }
}

TEST(PnpDlt, TestDlt22) {
    Eigen::Matrix3d const K{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};  // Pixels must be in normalized space
    MvgFrameGenerator const generator{MvgFrameGenerator(true, K)};

    for (size_t i{0}; i < 20; ++i) {
        MvgFrame const frame_i{generator.Generate()};

        auto const tf{Dlt22(frame_i.pixels, frame_i.points)};

        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant

        Eigen::Vector<double, 6> const pose_i{geometry::Log(tf)};
        EXPECT_TRUE(pose_i.isApprox(frame_i.pose)) << "Result:\n" << pose_i << "\nexpected result:\n" << frame_i.pose;
    }
}