#include "transforms/extrinsic.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(TransformsExtrinsic, TestHasPath) {
    // An empty extrinsic will not find a path between two different assets but it can still get the identity tf between
    // the same asset.
    transforms::Extrinsics value{{}};
    EXPECT_TRUE(value.HasPath(AssetId{1}, AssetId{1}));
    EXPECT_FALSE(value.HasPath(AssetId{1}, AssetId{2}));

    value = transforms::Extrinsics{{
        // These first two are connected!
        {0, 1, Array6d::Zero()},
        {2, 1, Array6d::Zero()},
        // This one is disconnected!
        {3, 4, Array6d::Zero()},
    }};
    EXPECT_TRUE(value.HasPath(AssetId{0}, AssetId{2}));
    EXPECT_FALSE(value.HasPath(AssetId{0}, AssetId{4}));
}

TEST(TransformsExtrinsic, TestFindPath) {
    std::vector<Extrinsic> extrinsics{};
    // Even for an empty extrinsic there is always a self transform returned for any two same frames.
    auto result{transforms::Extrinsics::FindPath(extrinsics, AssetId{1}, AssetId{1})};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(std::size(*result), 0);

    // Any non-same frames will not have a valid path given the empty intrinsic.
    result = transforms::Extrinsics::FindPath(extrinsics, AssetId{1}, AssetId{2});
    EXPECT_FALSE(result.has_value());

    extrinsics = {
        // These first two are connected!
        {0, 1, Array6d::Zero()},
        {2, 1, Array6d::Zero()},
        // This one is disconnected!
        {3, 4, Array6d::Zero()},
    };

    // Transform within a single extrinsic element.
    result = transforms::Extrinsics::FindPath(extrinsics, AssetId{1}, AssetId{2});
    ASSERT_TRUE(result.has_value());
    // The second extrinsic captures this relation directly which is why there is only one path element.
    EXPECT_EQ(std::size(*result), 1);
    // We asked for 2->1 but the extrinsic has 1->2 which is why 'forward' is false.
    EXPECT_FALSE(result->at(0).forward);

    // Transform across two extrinsic elements.
    result = transforms::Extrinsics::FindPath(extrinsics, AssetId{2}, AssetId{0});
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(std::size(*result), 2);
    // The first transform will need to take us from 0 to 1 therefore it is an inverse (i.e. forward==false).
    auto const& tf_0{result->at(0)};
    EXPECT_FALSE(tf_0.forward);
    EXPECT_EQ(tf_0.extrinsic.frame_a, AssetId{0});
    EXPECT_EQ(tf_0.extrinsic.frame_b, AssetId{1});
    // The second transform needs to take us from 1 to 2 and therefore it the given extrinsic is forward==true!
    auto const& tf_1{result->at(1)};
    EXPECT_TRUE(tf_1.forward);
    EXPECT_EQ(tf_1.extrinsic.frame_a, AssetId{2});
    EXPECT_EQ(tf_1.extrinsic.frame_b, AssetId{1});

    // Attempted transform across two disconnected extrinsic elements.
    result = transforms::Extrinsics::FindPath(extrinsics, AssetId{4}, AssetId{0});
    EXPECT_FALSE(result.has_value());
}