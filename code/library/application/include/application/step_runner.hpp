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
concept IsStep = requires(Result const result, Step const step, std::shared_ptr<database::CalibrationDatabase> db) {
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
std::pair<Result, CacheStatus> RunStep(Step const& step, std::shared_ptr<database::CalibrationDatabase> const db) {
    auto const cached_key{database::ReadCacheKey(db, step.step_type, step.SensorName())};
    std::string const new_key{step.CacheKey()};

    if (CacheHit(cached_key, new_key)) {
        return {step.Load(db), CacheStatus::CacheHit};
    } else {
        // There was no cache hit so we remove the current step (if it exists) which, assuming the "cascade on delete"
        // logic is correct, will also remove all other data with a foreign key dependency on this step. Then we write
        // the step new with the new cache key.
        database::RemoveFromDb(step.step_type, step.SensorName(), db);
        database::WriteToDb(step.step_type, new_key, step.SensorName(), db);
    }

    // ERROR(Jack): What if the compute step fails, but we already wrote the new cache key into the step table. If we
    // run the step again on the same database then it will think it was a cache hit even though the code did not
    // actually finish executing.
    Result const result{step.Compute()};
    step.Save(result, db);

    return {result, CacheStatus::CacheMiss};
}

}  // namespace reprojection::application
