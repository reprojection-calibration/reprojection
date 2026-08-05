#include "database_semantics.hpp"

#include <sqlite3.h>

// cppcheck-suppress missingInclude
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
}  // LCOV_EXCL_LINE

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

std::optional<StepId> ReadStepId(sqlite3* const db, StepType const type, Hash const& cache_key) {
    auto const binder{[type, cache_key](sqlite3_stmt* stmt) {
        Bind(stmt, 1, ToString(type));
        Bind(stmt, 2, cache_key.value);
    }};

    std::optional<StepId> data;
    ExecuteQuery(db, sql_statements::steps_select, binder, [&data](sqlite3_stmt* const stmt) {
        StepId const step_id{sqlite3_column_int64(stmt, 0)};

        data = step_id;
    });

    return data;
}

StepId InsertStep(sqlite3* const db, StepType const type) {
    auto const binder{[type](sqlite3_stmt* stmt) {
        Bind(stmt, 1, ToString(type));
        BindNull(stmt, 2);
    }};

    StepId data{-1};
    ExecuteQuery(db, sql_statements::steps_insert, binder,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}

}  // namespace reprojection::database