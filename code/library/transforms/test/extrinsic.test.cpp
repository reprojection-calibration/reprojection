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
