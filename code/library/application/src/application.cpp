#include "application/application.hpp"

#include "caching/cache_keys.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "database/sensor_data_interface_getters.hpp"

namespace reprojection::application {

std::pair<CacheStatus, CameraState> FocalLengthInitialization(
    CameraMeasurements const& targets, CameraInfo const& camera_info,
    std::shared_ptr<database::CalibrationDatabase> const database) {
    std::string const cache_key{caching::CacheKey(camera_info, targets)};
    CacheStatus const status{CacheState(database, "focal_length_initialization", cache_key)};

    // TODO(Jack): This logic of handling the step table cache key update should be generic for all cases!
    if (status == CacheStatus::StepNameMiss or status == CacheStatus::CacheKeyMiss) {
        if (status == CacheStatus::StepNameMiss) {
            database::AddCalibrationStep("focal_length_initialization", cache_key, database);
        } else {
            std::cout << "cache key update - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;
        }

        std::cout << "calculate the focal length initialization! - YOU STILL NEED TO IMPLEMENT THIS!!!" << std::endl;

        // ERROR WE JUST RETURN BLANK!
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

// TODO(Jack): Test!
// TODO(Jack): Better name than 'db_cache_key'?
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

}  // namespace reprojection::application