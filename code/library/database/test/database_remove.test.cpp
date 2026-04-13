#include "database/database_remove.hpp"

#include <gtest/gtest.h>

#include "database/calibration_database.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

class DatabaseRemoveFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = database::OpenCalibrationDatabase(":memory:", true, false);

        database::WriteToDb(camera_info, db);
    }

    SqlitePtr db{nullptr};
    CameraInfo camera_info{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds};
};

TEST_F(DatabaseRemoveFixture, TestRemoveFromDbStep) {
    // If there is no step to remove it is just silent
    EXPECT_NO_THROW(database::RemoveFromDb(CalibrationStep::Lpi, "", db));

    // Write a step to the database and load its cache key to check its there.
    database::WriteToDb(CalibrationStep::Lpi, "cache_key", camera_info.sensor_name, db);

    auto cache_key{database::ReadCacheKey(db, CalibrationStep::Lpi, camera_info.sensor_name)};
    ASSERT_TRUE(cache_key.has_value());

    // Remove the step and then try to load the cache key - but the cache key should be std::nullopt because the step
    // has been removed.
    EXPECT_NO_THROW(database::RemoveFromDb(CalibrationStep::Lpi, camera_info.sensor_name, db));

    cache_key = database::ReadCacheKey(db, CalibrationStep::Lpi, camera_info.sensor_name);
    EXPECT_FALSE(cache_key.has_value());
}
