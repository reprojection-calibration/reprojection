#include "application/application.hpp"

#include <gtest/gtest.h>

#include "caching/cache_keys.hpp"
#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "testing_utilities/constants.hpp"
#include "types/eigen_types.hpp"

using namespace reprojection;

TEST(ApplicationApplication, TestFocalLengthInitialization) {
    auto db{std::make_shared<database::CalibrationDatabase>(":memory:", true, false)};

    CameraMeasurements const camera_measurements{{0, {}}};
    CameraInfo const camera_info{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds};
    CameraState const gt_camera_state{testing_utilities::pinhole_intrinsics};

    // Artificially construct a representative calibration step entry with corresponding cache key.
    std::string const cache_key{caching::CacheKey(camera_info, camera_measurements)};
    database::AddCalibrationStep("focal_length_initialization", cache_key, db);
    AddIntrinsics(camera_info, gt_camera_state, "focal_length_initialization", db);

    auto const [cache_status,
                camera_state]{application::FocalLengthInitialization(camera_measurements, camera_info, db)};
    EXPECT_EQ(cache_status, application::CacheStatus::CacheHit);
    EXPECT_TRUE(camera_state.intrinsics.isApprox(gt_camera_state.intrinsics));
}

// NOTE(Jack): We use local namespaces and "in memory" databases to achieve complete isolation between the tests cases.
TEST(ApplicationApplication, TestFocalLengthInitializationCacheMisses) {
    CameraMeasurements const camera_measurements{{0, {}}};
    CameraInfo const camera_info{"/cam/retro/123", CameraModel::Pinhole, testing_utilities::image_bounds};
    {
        auto db{std::make_shared<database::CalibrationDatabase>(":memory:", true, false)};
        auto const [cache_status, _]{application::FocalLengthInitialization(camera_measurements, camera_info, db)};
        EXPECT_EQ(cache_status, application::CacheStatus::StepNameMiss);
    }
    {
        auto db{std::make_shared<database::CalibrationDatabase>(":memory:", true, false)};
        database::AddCalibrationStep("focal_length_initialization", "nonexistent_cache_key", db);

        auto const [cache_status, _]{application::FocalLengthInitialization(camera_measurements, camera_info, db)};
        EXPECT_EQ(cache_status, application::CacheStatus::CacheKeyMiss);
    }
}