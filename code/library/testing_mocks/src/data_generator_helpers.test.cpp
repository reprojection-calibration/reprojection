#include "data_generator_helpers.hpp"

#include <gtest/gtest.h>

#include "testing_utilities/constants.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

// To understand why 969 is the last valid timestamp take a look at the implementation of SampleTimes(). There you will
// at least see the math, but for it properly to make sense, you need to dig into the core spline
// implementation/literature.
TEST(TestingMocksDataGeneratorHelpers, TestTimedSphereTrajectorySpline) {
    // A spline with 100 control points is suitable for generating 50 evenly sampled data points. If you try to sample
    // more than that you will see artifacts!
    auto const se3_spline{testing_mocks::TimedSphereTrajectorySpline(100, 1000)};

    auto pose{se3_spline.Evaluate(0)};
    EXPECT_TRUE(pose.has_value());

    pose = se3_spline.Evaluate(969);
    EXPECT_TRUE(pose.has_value());

    pose = se3_spline.Evaluate(970);
    EXPECT_FALSE(pose.has_value());
}

TEST(TestingMocksDataGeneratorHelpers, TestSampleTimes) {
    std::set<uint64_t> const times{testing_mocks::SampleTimes(100, 1000)};

    for (auto const time: times) {
        std::cout << time << std::endl;
    }

    EXPECT_EQ(*std::cbegin(times), 0);
    EXPECT_EQ(*std::crbegin(times), 960);
}