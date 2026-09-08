#include "logging/fmt.hpp"

#include <gtest/gtest.h>

#include "types/transform_types.hpp"

using namespace reprojection;

TEST(LoggingFmt, TestFmtExtrinsic) {
    Extrinsic const data{AssetId{1}, AssetId{2}, Array6d::Ones()};

    std::string const result{fmt::format("{}", data)};

    EXPECT_EQ(result, "{'frame_a': 1, 'frame_b': 2, 'se3_a_b': [1, 1, 1, 1, 1, 1]}");
}

TEST(LoggingFmt, TestRigState) {
    Frames const rig_poses{Frame{1, Array6d::Ones()}, Frame{2, Array6d::Ones()}};
    Extrinsic const extrinsic_1{AssetId{1}, AssetId{2}, Array6d::Ones()};
    Extrinsic const extrinsic_2{AssetId{2}, AssetId{3}, Array6d::Ones()};
    transforms::RigState const data{AssetId{1}, rig_poses, transforms::Extrinsics{{extrinsic_1, extrinsic_2}}};

    std::string const result{fmt::format("{}", data)};

    EXPECT_EQ(result,
              "{'rig_frame_asset_id': 1, 'num_poses': 2, 'extrinsics': [{'frame_a': 1, 'frame_b': 2, 'se3_a_b': [1, 1, "
              "1, 1, 1, 1]}, {'frame_a': 2, 'frame_b': 3, 'se3_a_b': [1, 1, 1, 1, 1, 1]}]}");
}

TEST(LoggingFmt, TestBaCameraState) {
    optimization::BundleAdjustment::CameraState const data{Intrinsic{Array3d::Ones()}, Array6d::Ones()};

    std::string const result{fmt::format("{}", data)};

    EXPECT_EQ(result, "{'intrinsic': [1.000, 1.000, 1.000], 'extrinsic': [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]}");
}

TEST(LoggingFmt, TestBaResult) {
    // AssetId rig_frame_asset_id;
    // Frames rig_poses;
    // std::map<AssetId, CameraState> camera_states;
    Frames const rig_poses{Frame{1, Array6d::Ones()}, Frame{2, Array6d::Ones()}};
    std::map<AssetId, optimization::BundleAdjustment::CameraState> camera_states{
        {AssetId{1}, {Intrinsic{Array3d::Ones()}, Array6d::Ones()}},
        {AssetId{2}, {Intrinsic{Array3d::Ones()}, Array6d::Ones()}}};

    optimization::BundleAdjustment::Result const data{AssetId{1}, rig_poses, camera_states};

    std::string const result{fmt::format("{}", data)};

    EXPECT_EQ(
        result,
        "{'rig_frame_asset_id': 1, 'num_poses': 2, 'camera_states': [{'asset_id': 1, 'state': {'intrinsic': [1.000, "
        "1.000, 1.000], 'extrinsic': [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]}}, {'asset_id': 2, 'state': "
        "{'intrinsic': [1.000, 1.000, 1.000], 'extrinsic': [1.000, 1.000, 1.000, 1.000, 1.000, 1.000]}}]}");
}

TEST(LoggingFmt, TestCeresSolverSummary) {
    ceres::Solver::Summary const data;

    std::string const result{fmt::format("{}", data)};

    EXPECT_EQ(result,
              "{'initial_cost': -1.00, 'final_cost': -1.00, 'num_successful_steps': -1, "
              "'num_unsuccessful_steps': -1}");
}