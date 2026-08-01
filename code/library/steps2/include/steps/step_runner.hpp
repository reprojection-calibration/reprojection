#pragma once

#include "database/calibration_database.hpp"

namespace reprojection::steps {

// TODO(Jack): Where does this belong and do we really need this idea at all?
struct StepOwner {
    static StepOwner Recording(RecordingId const id) { return StepOwner{id, std::nullopt}; }

    static StepOwner Run(RunId const id) { return StepOwner{std::nullopt, id}; }

    std::optional<RecordingId> recording_id;
    std::optional<RunId> run_id;

   private:
    StepOwner(std::optional<RecordingId> const& _recording_id, std::optional<RunId> const& _run_id)
        : recording_id{_recording_id}, run_id{_run_id} {}
};

template <typename T>
concept IsRunnableStep = requires(T const& step, StepId const id, database::CalibrationDatabase& db) {
    { step.Type() } -> std::same_as<StepType>;
    { step.CacheKey() } -> std::same_as<Hash>;
    { step.Execute(id, db) } -> std::same_as<void>;
};

template <typename T>
    requires IsRunnableStep<T>
StepId RunStep(StepOwner const owner, T const& step, database::CalibrationDatabase& db) {
    Hash const cache_key{step.CacheKey()};

    auto const [step_id, cache_status]{db.GetOrCreateStep(owner.recording_id, owner.run_id, step.Type(), cache_key)};
    if (cache_status == CacheStatus::CacheHit) {
        return step_id;
    }

    // TODO(Jack): Put this inside a database transaction so in case of failure everything rolls back!
    step.Execute(step_id, db);
    db.StepCacheKeyUpdate(step_id, cache_key);

    return step_id;
}
}  // namespace reprojection::steps
