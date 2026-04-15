#pragma once

#include "database/database_read.hpp"
#include "database/database_remove.hpp"
#include "database/database_write.hpp"

namespace reprojection::steps {

inline bool CacheHit(std::optional<std::string> const& loaded_key, std::string_view key) {
    if (loaded_key.has_value() and loaded_key.value() == key) {
        return true;
    }

    return false;
}

template <typename Result, typename Step>
concept IsStep = requires(Result const result, Step const step, SqlitePtr const db) {
    { step.step_type } -> std::convertible_to<CalibrationStep>;
    { step.SensorName() } -> std::same_as<std::string>;
    { step.CacheKey() } -> std::same_as<std::string>;
    { step.Compute() } -> std::same_as<Result>;
    { step.Load(db) } -> std::same_as<Result>;
    { step.Save(result, db) };
};

// TODO(Jack): Make private one day when the application is whole
template <typename Result, typename Step>
    requires IsStep<Result, Step>
std::pair<Result, CacheStatus> RunStep(Step const& step, SqlitePtr const db) {
    auto const cached_key{database::ReadCacheKey(db, step.step_type, step.SensorName())};
    std::string const new_key{step.CacheKey()};

    if (CacheHit(cached_key, new_key)) {
        return {step.Load(db), CacheStatus::CacheHit};
    }

    Result const result{step.Compute()};

    // NOTE(Jack): We should explain a little but of what we are doing with the database here because it is really the
    // core caching logic.
    database::RemoveFromDb(step.step_type, step.SensorName(), db);
    database::WriteToDb(step.step_type, std::nullopt, step.SensorName(), db);

    step.Save(result, db);

    database::WriteToDb(step.step_type, new_key, step.SensorName(), db);

    return {result, CacheStatus::CacheMiss};
}

}  // namespace reprojection::steps
