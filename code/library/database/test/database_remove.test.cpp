#include "database/database_remove.hpp"

#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(DatabaseDatabaseRemove, TestRemoveFromDbStep) {
    auto const db{std::make_shared<database::CalibrationDatabase>(":memory:", true, false)};

    // If there is no step to remove it is just silent
    EXPECT_NO_THROW(database::RemoveFromDb(CalibrationStep::Lpi, "", db));

    // Write a step to the database and load its cache key to check its there.
    database::WriteToDb(CameraInfo{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds}, db);
    database::WriteToDb(CalibrationStep::Lpi, "cache_key", "/cam/retro/123", db);

    auto cache_key{database::ReadCacheKey(db, CalibrationStep::Lpi, "/cam/retro/123")};
    ASSERT_TRUE(cache_key.has_value());

    // Remove the step and then try to load the cache key - but the cache key should be std::nullopt because the step
    // has been removed.
    EXPECT_NO_THROW(database::RemoveFromDb(CalibrationStep::Lpi, "/cam/retro/123", db));

    cache_key = database::ReadCacheKey(db, CalibrationStep::Lpi, "/cam/retro/123");
    EXPECT_FALSE(cache_key.has_value());
}
