#pragma once

#include "database/database_read.hpp"
#include "database/database_remove.hpp"
#include "database/database_write.hpp"

namespace reprojection::application {

// TODO MOVE TO TYPES
enum class CacheStatus {
    CacheHit,
    CacheMiss,
};

// TODO MOVE TO TYPES
std::string ToString(CacheStatus const status) {
    if (status == CacheStatus::CacheHit) {
        return "cache_hit";
    } else if (status == CacheStatus::CacheMiss) {
        return "cache_miss";
    } else {
        throw std::runtime_error{"Library implementation error ToString(CacheStatus)"};
    }
}

// TODO NAMING!
CacheStatus Xxx(std::optional<std::string> const& loaded_key, std::string_view key) {
    if (loaded_key.has_value() and loaded_key.value() == key) {
        return CacheStatus::CacheHit;
    } else {
        return CacheStatus::CacheMiss;
    }
}

// TODO(Jack): Make private one day when the application is whole
// TODO ADD CONCEPTS!
template <typename Result, typename Step>
std::pair<Result, CacheStatus> RunStep(Step const& step, std::shared_ptr<database::CalibrationDatabase> const db) {
    auto const loaded_key{database::ReadCacheKey(db, step.step_type, step.SensorName())};
    std::string const new_key{step.CacheKey()};

    CacheStatus const status{Xxx(loaded_key, new_key)};
    if (status == CacheStatus::CacheHit) {
        return {step.Load(db), CacheStatus::CacheHit};
    } else {
        // Remove the calibration step - the "on cascade" relationships mean that this should remove all data from all
        // tables with a foreign key relationship on the removed step.
        database::RemoveFromDb(step.step_type, step.SensorName(), db);
        database::WriteToDb(step.step_type, new_key, step.SensorName(), db);
    }

    Result const result{step.Compute()};
    step.Save(result, db);

    return {result, CacheStatus::CacheMiss};
}

}  // namespace reprojection::application
