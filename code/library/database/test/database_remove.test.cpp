#include "database/database_remove.hpp"

#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

TEST(DatabaseDatabaseRemove, TestRemoveFromDb) {
    SqlitePtr db{database::OpenCalibrationDatabase(":memory:", true, false)};

    std::string const sensor_name{"/cam/retro/123"};
    auto const step_type{CalibrationStep::PoseInitialization};

    // If there is no step to remove it is just silent
    EXPECT_NO_THROW(database::RemoveFromDb(db, sensor_name, step_type));

    // Write a step to the database and load its cache key to check its there.
    database::InsertEntity(db, sensor_name, Entity::Camera);
    database::InsertStep(db, sensor_name, step_type, "cache_key");
    auto cache_key{database::ReadCacheKey(db, sensor_name, step_type)};
    ASSERT_TRUE(cache_key.has_value());

    // Remove the step and then try to load the cache key - but the cache key should be std::nullopt because the step
    // has been removed.
    EXPECT_NO_THROW(database::RemoveFromDb(db, sensor_name, step_type));

    cache_key = database::ReadCacheKey(db, sensor_name, step_type);
    EXPECT_FALSE(cache_key.has_value());
}
