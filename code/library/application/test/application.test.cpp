#include "application/application.hpp"

#include <gtest/gtest.h>

#include "caching/cache_keys.hpp"
#include "database/calibration_database.hpp"
#include "database/image_interface.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "database/sensor_data_interface_getters.hpp"
#include "testing_utilities/constants.hpp"
#include "types/eigen_types.hpp"

namespace reprojection::application {

enum class CacheStatus {
    CacheHit,
    CacheKeyMiss,
    StepNameMiss,
};

// TODO NAMING!!!!

CacheStatus CacheState(std::shared_ptr<database::CalibrationDatabase const> const database, std::string_view step_name,
                       std::string_view cache_key) {
    auto const db_cache_key{database::GetCacheKey(database, step_name)};

    if (not db_cache_key) {
        return CacheStatus::StepNameMiss;
    } else if (db_cache_key.value() == cache_key) {
        return CacheStatus::CacheHit;
    } else {
        return CacheStatus::CacheKeyMiss;
    }
}

// TODO(Jack): Right now there are two cache miss categories with similar logic! Eliminate the repetition!
std::pair<CacheStatus, CameraState> FocalLengthInitialization(
    CameraMeasurements const& targets, CameraInfo const& camera_info,
    std::shared_ptr<database::CalibrationDatabase> const database) {
    std::string const cache_key{caching::CacheKey(camera_info, targets)};
    CacheStatus const status{CacheState(database, "focal_length_initialization", cache_key)};

    if (status == CacheStatus::StepNameMiss or status == CacheStatus::CacheKeyMiss) {
        std::cout << "cache miss - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;

        if (status == CacheStatus::StepNameMiss) {
            database::AddCalibrationStep("focal_length_initialization", cache_key, database);
        } else {
            // TODO(Jack): Update the key of step with the new key!
            std::cout << "cache key update - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;
        }

        return {status, {}};
    } else if (status == CacheStatus::CacheHit) {
        auto const result{GetIntrinsic(database, "focal_length_initialization", camera_info.sensor_name)};
        if (not result.has_value()) {
            throw std::runtime_error("We need a real error handling strategy here!");
        }
        auto const [_, camera_state]{result.value()};

        return {status, camera_state};
    } else {
        throw std::runtime_error("Library implementation error - unknown CacheStatus!!!!");
    }
}

}  // namespace reprojection::application

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