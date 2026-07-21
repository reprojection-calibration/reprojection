#include "database/calibration_database.hpp"

#include <format>

#include "database/sqlite_exception.hpp"
#include "generated/sql.hpp"

#include "database_semantics.hpp"
#include "serialization.hpp"
#include "sqlite_helpers.hpp"

namespace reprojection::database {

CalibrationDatabase::CalibrationDatabase(fs::path const& db_path, bool const create, bool const read_only) {
    if (create and read_only) {
        throw std::runtime_error(
            "You requested to open a database object with both options 'create' and 'read_only' true. This is "
            "an invalid combination as creating a database requires writing to it!");
    }

    // TODO(Jack): Consider using sqlite3_errcode for better terminal output https://sqlite.org/c3ref/errcode.html

    int code;
    if (create) {
        code = sqlite3_open_v2(db_path.c_str(), &db_, SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE, nullptr);
    } else if (read_only) {
        code = sqlite3_open_v2(db_path.c_str(), &db_, SQLITE_OPEN_READONLY, nullptr);
    } else {
        code = sqlite3_open_v2(db_path.c_str(), &db_, SQLITE_OPEN_READWRITE, nullptr);
    }

    if (code != SQLITE_OK) {
        // TODO(Jack): Is it valid here to try to get an error message here from an improperly opened db pointer?
        throw SqliteException(db_);
    }

    if (not read_only) {
        ExecuteStatement(sql_statements::assets_table, db_);
        ExecuteStatement(sql_statements::extracted_targets_table, db_);
        ExecuteStatement(sql_statements::images_table, db_);
        ExecuteStatement(sql_statements::recordings_table, db_);
        ExecuteStatement(sql_statements::runs_table, db_);
        ExecuteStatement(sql_statements::steps_table, db_);
        ExecuteStatement(sql_statements::target_info_table, db_);
    }

    // NOTE(Jack): We use the foreign key constraint between some tables to enforce data consistency. For
    // example a row in initial_camera_poses can only possibly exist if there is a corresponding entry in
    // extracted_targets. And that row in the extracted_targets table can only possibly exist if there is a
    // corresponding entry in the images table.
    //
    // That being said sqlite has the foreign key option off by default (https://sqlite.org/foreignkeys.html) so
    // we need to manually turn it on here.
    ExecuteStatement("PRAGMA foreign_keys = ON;", db_);
}

AssetId CalibrationDatabase::GetOrCreateAsset(AssetType const type, size_t const index, Name const& name) {
    auto const result{ReadAssetId(db_, type, index)};
    if (result and result->second != name) {
        throw std::runtime_error(
            std::format("Asset of type '{}', index '{}' and name '{}' already exists - cannot change name '{}'.",
                        ToString(type), index, result->second.value, name.value));
    } else if (result) {
        return result->first;
    }

    return InsertAsset(db_, type, index, name);
}

RecordingId CalibrationDatabase::GetOrCreateRecording(Name const& name, Hash const& hash) {
    auto const result{ReadRecordingId(db_, name)};
    if (result and result->second != hash) {
        throw std::runtime_error(std::format("Recording '{}' with hash '{}' already exists - cannot change hash '{}'.",
                                             name.value, result->second.value, hash.value));
    } else if (result) {
        return result->first;
    }

    return InsertRecording(db_, name, hash);
}

RunId CalibrationDatabase::GetOrCreateRun(RecordingId const recording_id, std::string_view config) {
    // BUG(Jack)
    // BUG(Jack)
    // BUG(Jack)
    // BUG(Jack)
    // ERROR(Jack): Use a real hash functions!!!! See BUG below!
    Hash const config_hash{std::string(config)};

    // NOTE(Jack): Unlike when the Asset/Recording logic, we add new runs if the config changes, that is why we
    // don't have an error block here.
    auto const result{ReadRunId(db_, recording_id, config_hash)};
    // BUG(Jack)
    // BUG(Jack)
    // BUG(Jack)
    // BUG(Jack)
    // ERROR(Jack): We are comparing the actually loaded config to the provided hash value, if we actually used a hash
    // function this would always be false! This is a landmine!
    if (result and result->second == config_hash.value) {
        return result->first;
    }

    return InsertRun(db_, recording_id, config_hash, config);
}

std::pair<StepId, CacheStatus> CalibrationDatabase::GetOrCreateStep(std::optional<RecordingId> const& recording_id,
                                                                    std::optional<RunId> const& run_id,
                                                                    StepType const type, Hash cache_key) {
    auto const result{ReadStepId(db_, recording_id, run_id, type)};
    if (result and result->second == cache_key) {
        return std::make_pair(result->first, CacheStatus::CacheHit);
    } else if (result) {
        return std::make_pair(UpsertStep(db_, result->first, recording_id, run_id, type), CacheStatus::CacheMiss);
    }

    return std::make_pair(InsertStep(db_, recording_id, run_id, type), CacheStatus::CacheMiss);
}

// TODO(Jack): How do we handle the case when we are asked to complete a step which does not exist? Throw? Does it
// already do that?
void CalibrationDatabase::StepCacheKeyUpdate(StepId const step_id, Hash const& cache_key) {
    auto const binder{[step_id, cache_key](sqlite3_stmt* const stmt) {
        Bind(stmt, 1, cache_key.value);
        Bind(stmt, 2, step_id.value);
    }};

    ExecuteStatement(sql_statements::steps_update_cache_key, binder, db_);
}

void CalibrationDatabase::ImagesInsert(StepId const step_id, AssetId const asset_id, EncodedImages const& data) {
    auto const binder{[step_id, asset_id](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, buffer]{data_i};

        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, asset_id.value);
        Bind(stmt, 3, timestamp_ns);

        if (buffer.data.empty()) {
            BindNull(stmt, 4);
        } else {
            BindBlob(stmt, 4, std::as_bytes(std::span{buffer.data}));
        }
    }};

    BatchExecuteStatement(sql_statements::images_insert, data, binder, db_);
}

EncodedImages CalibrationDatabase::ImagesSelect(StepId const step_id, AssetId const asset_id) {
    EncodedImages data;

    ExecuteQuery(
        db_, sql_statements::images_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&data](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};

            auto const blob{SqliteBlob(stmt, 1)};
            std::span<uchar const> blob_span{reinterpret_cast<uchar const*>(blob.data()), blob.size()};
            std::vector<uchar> buffer(std::cbegin(blob_span), std::cend(blob_span));

            // TODO(Jack): Should we represent empty images with std::optional? Currently this will load all images,
            // and if the image is a null value it will just be a buffer with length zero.
            data.insert({timestamp_ns, ImageBuffer{buffer}});
        });

    return data;
}

void CalibrationDatabase::ExtractedTargetsInsert(StepId const step_id, StepId const source_step_id,
                                                 AssetId const asset_id, CameraMeasurements const& data) {
    auto const binder{[step_id, source_step_id, asset_id](sqlite3_stmt* const stmt, auto const& data_i) {
        auto const& [timestamp_ns, target]{data_i};

        protobuf_serialization::ExtractedTargetProto const serialized{Serialize(target)};
        std::string buffer;
        if (not serialized.SerializeToString(&buffer)) {
            throw std::runtime_error(
                std::format("ExtractedTargetProto.SerializeToString() failed: step_id '{}', source_step_id '{}', "
                            "asset_id '{}', timestamp_ns '{}'",
                            step_id.value, source_step_id.value, asset_id.value, timestamp_ns));
        }

        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, source_step_id.value);
        Bind(stmt, 3, asset_id.value);
        Bind(stmt, 4, timestamp_ns);
        BindBlob(stmt, 5, std::as_bytes(std::span{buffer}));
    }};

    BatchExecuteStatement(sql_statements::extracted_targets_insert, data, binder, db_);
}

CameraMeasurements CalibrationDatabase::ExtractedTargetsSelect(StepId const step_id, AssetId const asset_id) {
    CameraMeasurements data;

    ExecuteQuery(
        db_, sql_statements::extracted_targets_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&data](sqlite3_stmt* const stmt) {
            uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(stmt, 0))};

            auto const blob{SqliteBlob(stmt, 1)};
            protobuf_serialization::ExtractedTargetProto serialized;
            serialized.ParseFromArray(std::data(blob), static_cast<int>(std::size(blob)));

            auto const deserialized{Deserialize(serialized)};
            if (not deserialized) {
                throw std::runtime_error(std::format(
                    "ExtractedTargetProto.ParseFromArray()/Deserialize() failed: timestamp_ns '{}'", timestamp_ns));
            }

            data.insert({timestamp_ns, deserialized.value()});
        });

    return data;
}

void CalibrationDatabase::TargetInfoInsert(StepId const step_id, AssetId const asset_id,
                                           TargetInfo const& target_info) {
    auto const binder{[step_id, asset_id, target_info](sqlite3_stmt* const stmt) {
        Bind(stmt, 1, step_id.value);
        Bind(stmt, 2, asset_id.value);
        Bind(stmt, 3, ToString(target_info.target_type));
        Bind(stmt, 4, static_cast<int64_t>(target_info.height));
        Bind(stmt, 5, static_cast<int64_t>(target_info.width));
        Bind(stmt, 6, target_info.unit_dimension);
        Bind(stmt, 7, static_cast<int64_t>(target_info.asymmetric));
    }};

    ExecuteStatement(sql_statements::target_info_insert, binder, db_);
}

std::optional<TargetInfo> CalibrationDatabase::TargetInfoSelect(StepId const step_id, AssetId const asset_id) {
    std::optional<TargetInfo> target_info;

    ExecuteQuery(
        db_, sql_statements::target_info_select,
        [step_id, asset_id](sqlite3_stmt* const stmt) {
            Bind(stmt, 1, step_id.value);
            Bind(stmt, 2, asset_id.value);
        },
        [&target_info](sqlite3_stmt* const stmt) {
            TargetInfo result;
            result.target_type = ToTargetType(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 0)));
            result.height = sqlite3_column_int(stmt, 1);
            result.width = sqlite3_column_int(stmt, 2);
            result.unit_dimension = sqlite3_column_double(stmt, 3);
            result.asymmetric = static_cast<bool>(sqlite3_column_int(stmt, 4));

            target_info = result;
        });

    return target_info;
}

}  // namespace reprojection::database