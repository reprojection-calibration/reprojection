#include "calibration/calibration_utils.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(StepsBundleAdjustment, TestAlignRotations) {
    Vector6d const pose_1{1, 0, 0, 0, 0, 0};
    Vector6d const pose_2{-1 * pose_1};
    Frames const input{Frame{1, pose_1}, Frame{2, pose_2}};

    Frames const output{calibration::AlignRotations(input)};

    EXPECT_TRUE(output.at(1).value.isApprox(output.at(2).value));
}