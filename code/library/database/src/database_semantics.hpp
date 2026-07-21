#pragma once

#include <sqlite3.h>

#include <optional>

#include "types/database_types.hpp"

namespace reprojection::database {

std::optional<std::pair<AssetId, std::string>> ReadAssetId(sqlite3* const db, AssetType const type, size_t const index);

AssetId InsertAsset(sqlite3* const db, AssetType const type, size_t const index, std::string_view name);

// TODO(Jack): Should we define basic structs like Hash and Name? Passing around raw strings does not scale.
std::optional<std::pair<RecordingId, std::string>> ReadRecordingId(sqlite3* const db, std::string_view name);

RecordingId InsertRecording(sqlite3* const db, std::string_view name, std::string_view hash);

std::optional<std::pair<RunId, std::string>> ReadRunId(sqlite3* const db, RecordingId const recording_id,
                                                       std::string_view config_hash);

RunId InsertRun(sqlite3* const db, RecordingId const recording_id, std::string_view config_hash,
                std::string_view config);

std::optional<std::pair<StepId, std::string>> ReadStepId(sqlite3* const db,
                                                         std::optional<RecordingId> const& recording_id,
                                                         std::optional<RunId> const& run_id, StepType type);

StepId InsertStep(sqlite3* const db, std::optional<RecordingId> const& recording_id, std::optional<RunId> const& run_id,
                  StepType const type, std::string_view cache_key);

// NOTE(Jack): This is not strictly an upsert because we actually delete the entire row and then insert it again. We do
// this to make sure that "cascade on delete" operations happen. Official upsert semantics never call delete and
// therefore cannot be used here.
StepId UpsertStep(sqlite3* const db, StepId const id, std::optional<RecordingId> const& recording_id,
                  std::optional<RunId> const& run_id, StepType const type, std::string_view cache_key);

}  // namespace reprojection::database