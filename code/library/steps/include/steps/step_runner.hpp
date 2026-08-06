#pragma once

#include "database/calibration_database.hpp"
#include "logging/logging.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

template <typename T>
concept IsRunnableStep = requires(T const& step, StepId const id, SqlitePtr const db) {
    { step.Type() } -> std::same_as<StepType>;
    { step.CacheKey() } -> std::same_as<Hash>;
    { step.Execute(id, db) } -> std::same_as<void>;
};

template <typename T>
    requires IsRunnableStep<T>
StepId RunStep(WorkflowId const workflow_id, T const& step, SqlitePtr const db) {
    Hash const cache_key{step.CacheKey()};
    auto const [step_id, cache_status]{database::GetOrCreateStep(db.get(), step.Type(), cache_key)};

    // Regardless if it is a cache hit or miss we need to add it to the assigned workflow.
    database::WorkflowStepUpsert(db.get(), workflow_id, step.Type(), step_id);

    log->info("{{'cache_status': '{}', 'step_id': {:2}, 'step_type': '{}'}}", ToString(cache_status), step_id.value,
              ToString(step.Type()));

    if (cache_status == CacheStatus::CacheHit) {
        return step_id;
    }

    // TODO(Jack): Put this inside a database transaction so in case of failure everything rolls back!
    // TODO(Jack): Not just rollback, but if this throw then we get left with a step with a null cache key, how to
    // solve!?
    step.Execute(step_id, db);
    database::StepCacheKeyUpdate(db.get(), step_id, cache_key);

    return step_id;
}
}  // namespace reprojection::steps
