

#include "database_semantics.hpp"

#include <sqlite3.h>

#include "generated/sql.hpp"
#include "types/database_types.hpp"

#include "sqlite_helpers.hpp"

namespace reprojection::database {

std::optional<std::pair<AssetId, Name>> ReadAssetId(sqlite3* const db, AssetType const type, size_t const index) {
    auto const binder{[type, index](sqlite3_stmt* stmt) {
        Bind(stmt, 1, ToString(type));
        Bind(stmt, 2, index);
    }};

    std::optional<std::pair<AssetId, Name>> data;
    ExecuteQuery(db, sql_statements::assets_select, binder, [&data](sqlite3_stmt* const stmt) {
        AssetId const asset_id{sqlite3_column_int64(stmt, 0)};
        Name const name{std::string(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 1)))};

        data = std::make_pair(asset_id, name);
    });

    return data;
}

AssetId InsertAsset(sqlite3* const db, AssetType const type, size_t const index, Name const& name) {
    auto const binder{[type, index, name](sqlite3_stmt* stmt) {
        Bind(stmt, 1, ToString(type));
        Bind(stmt, 2, index);
        Bind(stmt, 3, name.value);
    }};

    AssetId data{-1};
    ExecuteQuery(db, sql_statements::assets_insert, binder,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}

// TODO(Jack): Should we define basic structs like Hash and Name? Passing around raw strings does not scale.
std::optional<std::pair<RecordingId, Hash>> ReadRecordingId(sqlite3* const db, Name const& name) {
    auto const binder{[name](sqlite3_stmt* stmt) { Bind(stmt, 1, name.value); }};

    std::optional<std::pair<RecordingId, Hash>> data;
    ExecuteQuery(db, sql_statements::recordings_select, binder, [&data](sqlite3_stmt* const stmt) {
        RecordingId const recording_id{sqlite3_column_int64(stmt, 0)};
        Hash const hash{std::string(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 1)))};

        data = std::make_pair(recording_id, hash);
    });

    return data;
}

RecordingId InsertRecording(sqlite3* const db, Name const& name, Hash const& hash) {
    auto const binder{[name, hash](sqlite3_stmt* stmt) {
        Bind(stmt, 1, name.value);
        Bind(stmt, 2, hash.value);
    }};

    RecordingId data{-1};
    ExecuteQuery(db, sql_statements::recordings_insert, binder,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}

std::optional<std::pair<RunId, std::string>> ReadRunId(sqlite3* const db, RecordingId const recording_id,
                                                       Hash const& config_hash) {
    auto const binder{[recording_id, config_hash](sqlite3_stmt* stmt) {
        Bind(stmt, 1, recording_id.value);
        Bind(stmt, 2, config_hash.value);
    }};

    std::optional<std::pair<RunId, std::string>> data;
    ExecuteQuery(db, sql_statements::runs_select, binder, [&data](sqlite3_stmt* const stmt) {
        RunId const run_id{sqlite3_column_int64(stmt, 0)};
        std::string const config{std::string(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 1)))};

        // TODO(Jack): Should we do some parsing checking to check that the config is properly formatted?
        data = std::make_pair(run_id, config);
    });

    return data;
}

RunId InsertRun(sqlite3* const db, RecordingId const recording_id, Hash const& config_hash, std::string_view config) {
    auto const binder{[recording_id, config_hash, config](sqlite3_stmt* stmt) {
        Bind(stmt, 1, recording_id.value);
        Bind(stmt, 2, config_hash.value);
        Bind(stmt, 3, config);
    }};

    RunId data{-1};
    ExecuteQuery(db, sql_statements::runs_insert, binder,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}

std::optional<std::pair<StepId, Hash>> ReadStepId(sqlite3* const db, std::optional<RecordingId> const& recording_id,
                                                  std::optional<RunId> const& run_id, StepType type) {
    auto const binder{[recording_id, run_id, type](sqlite3_stmt* stmt) {
        recording_id ? Bind(stmt, 1, recording_id->value) : BindNull(stmt, 1);
        run_id ? Bind(stmt, 2, run_id->value) : BindNull(stmt, 2);
        Bind(stmt, 3, ToString(type));
    }};

    std::optional<std::pair<StepId, Hash>> data;
    ExecuteQuery(db, sql_statements::steps_select, binder, [&data](sqlite3_stmt* const stmt) {
        StepId const step_id{sqlite3_column_int64(stmt, 0)};

        Hash cache_key;
        u_char const* const value{sqlite3_column_text(stmt, 1)};
        if (value) {
            cache_key.value = std::string(reinterpret_cast<char const*>(value));
        } else {
            cache_key = "6969";
        }

        data = std::make_pair(step_id, cache_key);
    });

    return data;
}

StepId InsertStep(sqlite3* const db, std::optional<RecordingId> const& recording_id, std::optional<RunId> const& run_id,
                  StepType const type) {
    auto const binder{[recording_id, run_id, type](sqlite3_stmt* stmt) {
        recording_id ? Bind(stmt, 1, recording_id->value) : BindNull(stmt, 1);
        run_id ? Bind(stmt, 2, run_id->value) : BindNull(stmt, 2);
        Bind(stmt, 3, ToString(type));
        BindNull(stmt, 4);
    }};

    StepId data{-1};
    ExecuteQuery(db, sql_statements::steps_insert, binder,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}

// TODO(Jack): We need a way better name for this function!
// NOTE(Jack): This is not strictly an upsert because we actually delete the entire row and then insert it again. We do
// this to make sure that "cascade on delete" operations happen. Official upsert semantics never call delete and
// therefore cannot be used here.
StepId UpsertStep(sqlite3* const db, StepId const id, std::optional<RecordingId> const& recording_id,
                  std::optional<RunId> const& run_id, StepType const type) {
    auto const binder1{[id](sqlite3_stmt* stmt) { Bind(stmt, 1, id.value); }};

    StepId data{-1};
    ExecuteStatement(sql_statements::steps_delete, binder1, db);

    auto const binder2{[id, recording_id, run_id, type](sqlite3_stmt* stmt) {
        Bind(stmt, 1, id.value);
        recording_id ? Bind(stmt, 2, recording_id->value) : BindNull(stmt, 2);
        run_id ? Bind(stmt, 3, run_id->value) : BindNull(stmt, 3);
        Bind(stmt, 4, ToString(type));
        BindNull(stmt, 5);
    }};

    // NOTE(Jack): Technically we know the id already so there is nothing that forces us to read it from the result of
    // this operation, but it is the pattern we use everywhere else and also its good to make sure what the database
    // actually processes, not just what we hope it does.
    ExecuteQuery(db, sql_statements::steps_insert_id, binder2,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}

}  // namespace reprojection::database