#pragma once

#include <sqlite3.h>

#include <optional>

#include "types/database_types.hpp"

namespace reprojection::database {

std::optional<std::pair<AssetId, Name>> ReadAssetId(sqlite3* db, AssetType type, size_t index);

AssetId InsertAsset(sqlite3* db, AssetType type, size_t index, Name const& name);

std::optional<WorkflowId> ReadWorkflowId(sqlite3* db, WorkflowType type, std::string_view signature);

WorkflowId InsertWorkflowId(sqlite3* db, WorkflowType type, std::string_view signature);

std::optional<StepId> ReadStepId(sqlite3* db, StepType type, Hash const& cache_key);

StepId InsertStep(sqlite3* db, StepType type);

}  // namespace reprojection::database