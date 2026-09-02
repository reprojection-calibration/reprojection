#include "calibration/calibration_utils.hpp"

#include <gtest/gtest.h>

#include <ranges>

using namespace reprojection;

TEST(CalibrationCalibrationUtils, TestAlignRotations) {
    Vector6d const pose_1{1, 0, 0, 0, 0, 0};
    Vector6d const pose_2{-1 * pose_1};
    Frames const input{Frame{1, pose_1}, Frame{2, pose_2}};

    Frames const output{calibration::AlignRotations(input)};

    EXPECT_TRUE(output.at(1).pose.isApprox(output.at(2).pose));
}

TEST(CalibrationCalibrationUtils, TestSynchronizeFrames) {
    auto const pose{Array6d::Random(6)};

    // All three match.
    Frames const data_x{{0, {pose}}, {10, {pose}}, {20, {pose}}};
    auto [frames_a, frames_b]{calibration::SynchronizeFrames(data_x, data_x, 2)};

    std::vector<uint64_t> expected{0, 10, 20};
    EXPECT_TRUE(std::ranges::equal(frames_a | std::views::keys, expected));
    EXPECT_TRUE(std::ranges::equal(frames_b | std::views::keys, expected));

    // The middle value of data_y does not match data_x and the others are with the synx tolerance so we only have two
    // matches.
    Frames const data_y{{1, {pose}}, {13, {pose}}, {19, {pose}}};
    std::tie(frames_a, frames_b) = calibration::SynchronizeFrames(data_x, data_y, 2);

    expected = {0, 20};
    EXPECT_TRUE(std::ranges::equal(frames_a | std::views::keys, expected));
    EXPECT_TRUE(std::ranges::equal(frames_b | std::views::keys, expected));

    // All data is outside of the sync tolerance so we get no matches at all.
    Frames const data_z{{3, {pose}}, {13, {pose}}, {23, {pose}}};
    std::tie(frames_a, frames_b) = calibration::SynchronizeFrames(data_x, data_z, 2);

    expected = {};
    EXPECT_TRUE(std::ranges::equal(frames_a | std::views::keys, expected));
    EXPECT_TRUE(std::ranges::equal(frames_b | std::views::keys, expected));
}