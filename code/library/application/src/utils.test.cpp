#include "utils.hpp"

#include <gtest/gtest.h>

#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(ApplicationUtils, TestAlignRotations) {
    Vector6d const pose_1{1, 0, 0, 0, 0, 0};
    Vector6d const pose_2{-1 * pose_1};
    OptimizationState const input{{}, {Frame{1, pose_1}, Frame{2, pose_2}}};

    OptimizationState const output{application::AlignRotations(input)};

    EXPECT_TRUE(output.frames.at(1).pose.isApprox(output.frames.at(2).pose));
}