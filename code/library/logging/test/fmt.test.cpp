#include "logging/fmt.hpp"

#include <gtest/gtest.h>

#include "types/transform_types.hpp"

using namespace reprojection;

TEST(LoggingLogging, TestFmtExtrinsic) {
    Extrinsic const data{AssetId{1}, AssetId{2}, Array6d::Ones()};

    std::string const result{fmt::format("{}", data)};

    EXPECT_EQ(result, "{'frame_a': 1, 'frame_b': 2, 'se3_a_b': [1, 1, 1, 1, 1, 1]}");
}

TEST(LoggingLogging, TestRigState) {
    Frames const rig_poses{Frame{1, Array6d::Ones()}, Frame{2, Array6d::Ones()}};
    Extrinsic const extrinsic_1{AssetId{1}, AssetId{2}, Array6d::Ones()};
    Extrinsic const extrinsic_2{AssetId{2}, AssetId{3}, Array6d::Ones()};
    transforms::RigState const data{AssetId{1}, rig_poses, transforms::Extrinsics{{extrinsic_1, extrinsic_2}}};

    std::string const result{fmt::format("{}", data)};

    EXPECT_EQ(result,
              "{'rig_frame_asset_id': 1, 'num_poses': 2, 'extrinsics': [{'frame_a': 1, 'frame_b': 2, 'se3_a_b': [1, 1, "
              "1, 1, 1, 1]}, {'frame_a': 2, 'frame_b': 3, 'se3_a_b': [1, 1, 1, 1, 1, 1]}]}");
}