#include "dlt.hpp"

#include <gtest/gtest.h>

#include "geometry/lie.hpp"
#include "testing_mocks/mvg_generator.hpp"

// TODO(Jack): I think we could add a test where we check more properties, like for example PC=0, etc. Even though these
// might already be checked in some sub-tests for specific test data, we should be able to do it for all executions.

using namespace reprojection;
using namespace reprojection::pnp;

TEST(PnpDlt, TestDlt23) {
    testing_mocks::MvgGenerator const generator{testing_mocks::MvgGenerator(false)};
    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};
        auto const [tf, K]{Dlt23(frame_i.pixels, frame_i.points)};

        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant
        EXPECT_TRUE(tf.isApprox(frame_i.pose)) << "Result:\n"
                                               << tf.matrix() << "\nexpected result:\n"
                                               << frame_i.pose.matrix();

        EXPECT_TRUE(K.isApprox(generator.GetK()));
    }
}

TEST(PnpDlt, TestDlt22) {
    Array4d const K{1, 1, 0, 0};  // Equivalent to K = I_3x3 Pixels must be in normalized image space for Dlt22
    testing_mocks::MvgGenerator const generator{
        testing_mocks::MvgGenerator(true, K)};  // Points must have Z=0 (flat = true)

    for (size_t i{0}; i < 20; ++i) {
        testing_mocks::MvgFrame const frame_i{generator.Generate(static_cast<double>(i) / 20)};

        auto const tf{Dlt22(frame_i.pixels, frame_i.points)};

        EXPECT_FLOAT_EQ(tf.linear().determinant(), 1);  // Property of rotation matrix - positive one determinant

        EXPECT_TRUE(tf.isApprox(frame_i.pose)) << "Result:\n"
                                               << tf.matrix() << "\nexpected result:\n"
                                               << frame_i.pose.matrix();
    }
}
