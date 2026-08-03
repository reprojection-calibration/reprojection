#pragma once

#include "database/calibration_database.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

template <typename T>
concept IsRunnableStep = requires(T const& step, StepId const id, database::CalibrationDatabase& db) {
    { step.Type() } -> std::same_as<StepType>;
    { step.CacheKey() } -> std::same_as<Hash>;
    { step.Execute(id, db) } -> std::same_as<void>;
};

template <typename T>
    requires IsRunnableStep<T>
StepId RunStep(T const& step, database::CalibrationDatabase& db) {
    Hash const cache_key{step.CacheKey()};
    auto const [step_id, cache_status]{db.GetOrCreateStep(step.Type(), cache_key)};

    log->info("{{'step_type': '{}', 'cache_status': '{}', 'step_id': {}, 'cache_key': '{}'}}", ToString(step.Type()),
              ToString(cache_status), step_id.value, cache_key.value);

    if (cache_status == CacheStatus::CacheHit) {
        return step_id;
    }

    // TODO(Jack): Put this inside a database transaction so in case of failure everything rolls back!
    step.Execute(step_id, db);
    db.StepCacheKeyUpdate(step_id, cache_key);

    return step_id;
}
}  // namespace reprojection::steps
