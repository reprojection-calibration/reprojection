#pragma once

#include "database/database_read.hpp"
#include "database/database_remove.hpp"
#include "database/database_write.hpp"

namespace reprojection::application {

inline bool CacheHit(std::optional<std::string> const& loaded_key, std::string_view key) {
    if (loaded_key.has_value() and loaded_key.value() == key) {
        return true;
    } else {
        return false;
    }
}

template <typename Result, typename Step>
concept CalibrationStepObject =
    requires(Result const result, Step const step, std::shared_ptr<database::CalibrationDatabase> db) {
        { step.step_type } -> std::convertible_to<CalibrationStep>;
        { step.SensorName() } -> std::same_as<std::string>;
        { step.CacheKey() } -> std::same_as<std::string>;
        { step.Compute() } -> std::same_as<Result>;
        { step.Load(db) } -> std::same_as<Result>;
        { step.Save(result, db) };
    };

// TODO(Jack): Make private one day when the application is whole
template <typename Result, typename Step>
    requires CalibrationStepObject<Result, Step>
std::pair<Result, CacheStatus> RunStep(Step const& step, std::shared_ptr<database::CalibrationDatabase> const db) {
    auto const loaded_key{database::ReadCacheKey(db, step.step_type, step.SensorName())};
    std::string const new_key{step.CacheKey()};

    if (CacheHit(loaded_key, new_key)) {
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
