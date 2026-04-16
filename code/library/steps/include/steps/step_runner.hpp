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

    // NOTE(Jack): The cache key logic here looks a little messy, and I think a sign that we are doing something wrong.
    // Combined with the underlying database logic it means I need to explain what's going on here. First of all if we
    // are here this means that it was not a cache hit and we have a new result that we want to write to the db. The
    // steps below are as follows:
    //
    // (1) In the first step here we RemoveFromDb() the step, and because we have ON DELETE CASCADE set
    // for all dependent data, and PRAGMA foreign_keys = ON, this should mean that the step itself and all the data
    // calculated in that step are removed.
    // (2) Then we write the step to the db again, but with a null cache key, because we have a foreign key constraint
    // that requires the step exists before we can Save() the data. The reason we leave it with a null key is that if
    // the Save() fails for whatever reason, then the next run will show as a cache failure because the key is null.
    // (3) Save the data - this can fail for whatever reason.
    // (4) Add the cache key to the step (remember the calibration_steps table has "upsert" semantics). Now the step is
    // actually cached.
    //
    // If there is a more clean way to express this logic, that would be a welcome addition. I think that the current
    // implementation can lead to problems in the future.
    database::RemoveFromDb(step.step_type, step.SensorName(), db);
    database::WriteToDb(step.step_type, std::nullopt, step.SensorName(), db);

    step.Save(result, db);

    database::WriteToDb(step.step_type, new_key, step.SensorName(), db);

    return {result, CacheStatus::CacheMiss};
}

}  // namespace reprojection::steps
