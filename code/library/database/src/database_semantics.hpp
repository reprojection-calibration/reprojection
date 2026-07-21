#pragma once

#include <sqlite3.h>

#include <optional>

#include "types/database_types.hpp"

namespace reprojection::database {

std::optional<std::pair<AssetId, Name>> ReadAssetId(sqlite3* const db, AssetType const type, size_t const index);

AssetId InsertAsset(sqlite3* const db, AssetType const type, size_t const index, Name const& name);

// TODO(Jack): Should we define basic structs like Hash and Name? Passing around raw strings does not scale.
std::optional<std::pair<RecordingId, Hash>> ReadRecordingId(sqlite3* const db, Name const& name);

RecordingId InsertRecording(sqlite3* const db, Name const& name, Hash const& hash);

std::optional<std::pair<RunId, std::string>> ReadRunId(sqlite3* const db, RecordingId const recording_id,
                                                       Hash const& config_hash);

// TODO(Jack): Use real config type!
RunId InsertRun(sqlite3* const db, RecordingId const recording_id, Hash const& config_hash, std::string_view config);

std::optional<std::pair<StepId, Hash>> ReadStepId(sqlite3* const db, std::optional<RecordingId> const& recording_id,
                                                  std::optional<RunId> const& run_id, StepType type);

StepId InsertStep(sqlite3* const db, std::optional<RecordingId> const& recording_id, std::optional<RunId> const& run_id,
                  StepType const type);

// TODO(Jack): We need a way better name here!
// NOTE(Jack): This is not strictly an upsert because we actually delete the entire row and then insert it again. We do
// this to make sure that "cascade on delete" operations happen. Official upsert semantics never call delete and
// therefore cannot be used here.
StepId UpsertStep(sqlite3* const db, StepId const id, std::optional<RecordingId> const& recording_id,
                  std::optional<RunId> const& run_id, StepType const type);

}  // namespace reprojection::database