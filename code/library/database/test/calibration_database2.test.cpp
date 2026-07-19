#include "database/calibration_database2.hpp"

#include <gtest/gtest.h>

#include <string>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(Yyy, TestGetOrCreateAsset) {
    TemporaryFile const temp_file{".db3"};

    auto db{database::CalibrationDatabase(temp_file.Path(), true)};

    // Repeated insert returns the same id with no problems.
    database::AssetId result{db.GetOrCreateAsset(database::AssetType::Camera, 0, "/cam0/image_raw")};
    EXPECT_EQ(result, database::AssetId{1});
    result = db.GetOrCreateAsset(database::AssetType::Camera, 0, "/cam0/image_raw");
    EXPECT_EQ(result, database::AssetId{1});

    // Add a second camera at index 1, no problemo.
    result = db.GetOrCreateAsset(database::AssetType::Camera, 1, "/cam1/image_raw");
    EXPECT_EQ(result, database::AssetId{2});

    // Adding an imu with an already used index is also no problem,
    result = db.GetOrCreateAsset(database::AssetType::Imu, 0, "/imu0");
    EXPECT_EQ(result, database::AssetId{3});

    // Trying to insert an asset with a different name at an already existing index is a no-go!
    EXPECT_THROW(db.GetOrCreateAsset(database::AssetType::Camera, 0, "/cam1/image_raw"), std::runtime_error);
    // Trying to create a new asset with an already existing name is also a no-go!
    EXPECT_THROW(db.GetOrCreateAsset(database::AssetType::Camera, 2, "/cam1/image_raw"), std::runtime_error);
    // Even if the asset type changes you are still not allowed to reuse a name!
    EXPECT_THROW(db.GetOrCreateAsset(database::AssetType::Imu, 2, "/cam1/image_raw"), std::runtime_error);
}
