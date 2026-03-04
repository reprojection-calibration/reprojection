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
    CacheKeyCacheMiss,
    StepNameCacheMiss,
};

// TODO NAMING!!!!

// TODO(Jack): Right now there are two cache miss categories with similar logic! Eliminate the repetition!
std::pair<CacheStatus, CameraState> FocalLengthInitialization(
    CameraMeasurements const& targets, CameraInfo const& camera_info,
    std::shared_ptr<database::CalibrationDatabase> const database) {
    auto const existing_cache_key{database::GetCacheKey(database, "focal_length_initialization")};

    if (not existing_cache_key.has_value()) {
        std::cout << "step_name cache miss - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;
        // here we will make a new calibration step entry
        return {CacheStatus::StepNameCacheMiss, {}};
    }

    std::string const new_cache_key{caching::CacheKey(camera_info, targets)};
    if (new_cache_key == existing_cache_key) {
        auto const result{GetIntrinsic(database, "focal_length_initialization", camera_info.sensor_name)};
        if (not result.has_value()) {
            throw std::runtime_error("We need a real error handling strategy here!");
        }

        auto const [_, camera_state]{result.value()};

        return {CacheStatus::CacheHit, camera_state};
    } else {
        std::cout << "key cache miss - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;
        // here we will update an existing calibration step entry
        return {CacheStatus::CacheKeyCacheMiss, {}};
    }
}

}  // namespace reprojection::application

using namespace reprojection;

TEST(ApplicationApplication, TestFocalLengthInitialization) {
    auto db{std::make_shared<database::CalibrationDatabase>(":memory:", true, false)};

    std::string const sensor_name{"/cam/retro/123"};
    uint64_t const timestamp_ns{0};
    CameraMeasurements const camera_measurements{{timestamp_ns, {}}};
    CameraInfo const camera_info{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds};

    auto const [cache_status, intrinsics]{application::FocalLengthInitialization(camera_measurements, camera_info, db)};

    EXPECT_EQ(cache_status, application::CacheStatus::StepNameCacheMiss);

    EXPECT_EQ(1, 2);
}