#pragma once

#include "database/calib_db.hpp"
#include "logging/fmt.hpp"
#include "logging/logging.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

namespace {

auto const log{logging::Get("steps")};

}

template <typename T>
concept IsRunnableStep = requires(T const& step, StepId const id, SqlitePtr const db) {
    { step.Type() } -> std::same_as<StepType>;
    { step.Assets() } -> std::same_as<std::vector<AssetId>>;
    { step.CacheKey() } -> std::same_as<Hash>;
    { step.Execute(id, db) } -> std::same_as<void>;
};

template <typename T>
    requires IsRunnableStep<T>
StepId RunStep(WorkflowId const workflow_id, T const& step, SqlitePtr const db) {
    Hash const cache_key{step.CacheKey()};
    auto const [step_id, cache_status]{database::GetOrCreateStep(db.get(), step.Type(), cache_key)};

    // Regardless if it is a cache hit or miss we need to add it to the assigned workflow.
    database::WorkflowStepUpsert(db.get(), workflow_id, step_id, step.Type(), step.Assets());

    log->info("\033[34m{{{}, 'cache_status': '{}'}}\033[0m", StepLogInfo{step.Type(), step_id}, ToString(cache_status));

    if (cache_status == CacheStatus::CacheHit) {
        return step_id;
    }

    // NOTE(Jack): This is almost the only place in the entire code base where we use exceptions for actual control
    // flow. If the step fails during execution then delete it from the steps table, and the FK relationships should
    // remove the entire record of the step from the database.
    try {
        step.Execute(step_id, db);
    } catch (std::exception const& e) {
        database::StepDelete(db.get(), step_id);

        log->error("{{{}, 'msg': {}}}", StepLogInfo{step.Type(), step_id}, std::string{e.what()});
        throw;
    }

    database::StepCacheKeyUpdate(db.get(), step_id, cache_key);

    return step_id;
}
}  // namespace reprojection::steps
