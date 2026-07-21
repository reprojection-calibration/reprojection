#pragma once

#include <sqlite3.h>

#include <filesystem>
#include <format>
#include <optional>

#include "database/serialization.hpp"
#include "generated/sql2.hpp"
#include "types/calibration_types.hpp"
#include "types/sensor_data_types.hpp"

namespace reprojection::database {

namespace fs = std::filesystem;

// NOTE(Jack): sqlite only supports int64_t as the int type, so instead of static casting later we just set the ids
// directly as this type.
struct AssetId {
    int64_t value;

    friend constexpr bool operator==(AssetId const&, AssetId const&) = default;
};

struct RecordingId {
    int64_t value;

    friend constexpr bool operator==(RecordingId const&, RecordingId const&) = default;
};

struct RunId {
    int64_t value;

    friend constexpr bool operator==(RunId const&, RunId const&) = default;
};

struct StepId {
    int64_t value;

    friend constexpr bool operator==(StepId const&, StepId const&) = default;
};

enum class AssetType { Camera, Imu, Target };

enum class StepType {
    FeatureExtraction,
    ImageLoading,
    ImuDataLoading,
    TargetInfo,
};

std::string ToString(AssetType const data) {
    if (data == AssetType::Camera) {
        return "camera";
    } else if (data == AssetType::Imu) {
        return "imu";
    } else if (data == AssetType::Target) {
        return "target";
    } else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unknown AssetType");
    }
}

inline std::string ToString(StepType const data) {
    if (data == StepType::FeatureExtraction) {
        return "feature_extraction";
    } else if (data == StepType::ImageLoading) {
        return "image_loading";
    } else if (data == StepType::ImuDataLoading) {
        return "imu_data_loading";
    } else if (data == StepType::TargetInfo) {
        return "target_info";
    }else {
        throw std::runtime_error("LIBRARY IMPLEMENTATION ERROR - Unknown StepType");
    }
}

// TODO(Jack): Add nullptr checks!
class SqliteException : public std::runtime_error {  // LCOV_EXCL_LINE
   public:
    SqliteException(sqlite3* const db, std::string_view sql) : std::runtime_error(FormatMessage(db, sql)) {}

    SqliteException(sqlite3* const db, sqlite3_stmt* const stmt) : std::runtime_error(FormatMessage(db, stmt)) {}

    SqliteException(sqlite3* const db) : std::runtime_error(FormatMessage(db)) {}

    SqliteException(sqlite3_stmt* const stmt) : std::runtime_error(FormatMessage(stmt)) {}

   private:
    static std::string FormatMessage(sqlite3* const db, std::string_view sql) {
        return "\n[SQLite Exception]\n"
               "----------------------------------------\n"
               "SQL Query:\n" +
               Indent(sql) + "\n" + "Error Code : " + std::to_string(sqlite3_errcode(db)) + "\n" +
               "Error Msg  : " + std::string(sqlite3_errmsg(db)) + "\n" + "----------------------------------------";
    }

    static std::string FormatMessage(sqlite3* const db, sqlite3_stmt* const stmt) {
        // TODO(Jack): Add RAII handler for expanded?
        char* const expanded{sqlite3_expanded_sql(stmt)};
        std::string const sql{expanded != nullptr ? std::string{expanded} : std::string{sqlite3_sql(stmt)}};
        sqlite3_free(expanded);

        return FormatMessage(db, sql);
    }

    static std::string FormatMessage(sqlite3* const db) {
        // TODO(Jack): Can we combine this with the full error message somehow so we do not need to copy and paste the
        // formatting twice?
        return "\n[SQLite Exception]\n"
               "----------------------------------------\n"
               "Error Code : " +
               std::to_string(sqlite3_errcode(db)) + "\n" + "Error Msg  : " + std::string(sqlite3_errmsg(db)) + "\n" +
               "----------------------------------------";
    }

    static std::string FormatMessage(sqlite3_stmt* const stmt) {
        // TODO(Jack): Add RAII handler for expanded?
        char* const expanded{sqlite3_expanded_sql(stmt)};
        std::string const sql{expanded != nullptr ? std::string{expanded} : std::string{sqlite3_sql(stmt)}};
        sqlite3_free(expanded);

        // TODO(Jack): Can we combine this with the full error message somehow so we do not need to copy and paste the
        // formatting twice?
        return "\n[SQLite Exception]\n"
               "----------------------------------------\n"
               "SQL Query:\n" +
               Indent(sql) + "\n" + "----------------------------------------";
    }

    static std::string Indent(std::string_view text) {
        std::string result;
        result.reserve(text.size() + 16);

        result += "  ";
        for (char c : text) {
            result += c;
            if (c == '\n') {
                result += "  ";
            }
        }

        return result;
    }  // LCOV_EXCL_LINE
};

class SqlStatement {
   public:
    SqlStatement(sqlite3* const db, char const* const sql) {
        if (sqlite3_prepare_v2(db, sql, -1, &stmt_, nullptr) != SQLITE_OK) {
            throw SqliteException(db, sql);
        }
    }

    ~SqlStatement() { sqlite3_finalize(stmt_); }

    sqlite3_stmt* stmt_{nullptr};
};

void Bind(sqlite3_stmt* const stmt, int const index, std::string_view value) {
    if (sqlite3_bind_text(stmt, index, std::string(value).c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) {
        throw SqliteException(stmt);
    }
}

void Bind(sqlite3_stmt* const stmt, int const index, int64_t const value) {
    if (sqlite3_bind_int64(stmt, index, value) != SQLITE_OK) {
        throw SqliteException(stmt);
    }
}

static void BindNull(sqlite3_stmt* const stmt, int const index) {
    if (sqlite3_bind_null(stmt, index) != SQLITE_OK) {
        throw SqliteException(stmt);
    }
}

// NOTE(Jack): We use SQLITE_TRANSIENT here because the serialized buffers in the lambdas disappear when the lambda
// is finished. Therefore, we want sql to make its own copy of the buffer when we call bind (i.e. SQLITE_TRANSIENT),
// so that way the external lifetime management can be disregarded.
static void BindBlob(sqlite3_stmt* const stmt, int const index, std::span<std::byte const> const& blob) {
    if (sqlite3_bind_blob(stmt, index, std::data(blob), std::size(blob), SQLITE_TRANSIENT) != SQLITE_OK) {
        throw SqliteException(stmt);
    }
}

bool StepRow(sqlite3_stmt* const stmt) {
    int const code{sqlite3_step(stmt)};

    if (code == SQLITE_ROW) {
        return true;
    } else if (code == SQLITE_DONE) {
        return false;
    } else {
        // TODO(Jack): We should really query the db for the real error message! Can we do this with just the stmt?
        throw std::runtime_error("SQLite step row failed");
    }
}

template <typename Binder, typename RowFunc>
void ExecuteQuery(sqlite3* const db, std::string_view sql, Binder&& binder, RowFunc&& on_row) {
    SqlStatement stmt{db, std::string(sql).data()};

    try {
        // NOTE(Jack): If the sql query statement does not use any dynamic binding (i.e. we want to perform a static
        // operation like creating a table), then we do not need to call the binder. Therefore, this code lets the user
        // pass in a nullptr for the binder, and it will then not execute any binding call. If this is actually possible
        // for people to understand is not clear at this time :)
        if constexpr (not std::is_same_v<std::decay_t<Binder>, std::nullptr_t>) {
            binder(stmt.stmt_);
        }
    } catch (...) {                                               // LCOV_EXCL_LINE
        std::throw_with_nested(SqliteException(db, stmt.stmt_));  // LCOV_EXCL_LINE
    }

    try {
        while (StepRow(stmt.stmt_)) {
            on_row(stmt.stmt_);
        }
    } catch (...) {                                               // LCOV_EXCL_LINE
        std::throw_with_nested(SqliteException(db, stmt.stmt_));  // LCOV_EXCL_LINE
    }
}

template <typename Binder>
void ExecuteStatement(std::string_view sql, Binder&& binder, sqlite3* const db) {
    SqlStatement stmt{db, std::string(sql).c_str()};

    try {
        binder(stmt.stmt_);
    } catch (...) {
        // TODO(Jack): It think it can very well be that any error thrown from bind is actually not 100% sqlite related,
        //  but actually due a error in the user code. Therefore it might be a mistake here to throw away the thrown
        //  error and replace it here with a database centric error. Think about also throwing the original error too!
        throw SqliteException(db, stmt.stmt_);
    }

    if (sqlite3_step(stmt.stmt_) != SQLITE_DONE) {
        throw SqliteException(db, stmt.stmt_);
    }
}

// Used for cases that do not require dynamic binding - passes an empty lambda which is a no-op.
inline void ExecuteStatement(std::string_view sql, sqlite3* const db) {
    ExecuteStatement(sql, [](sqlite3_stmt*) {}, db);
}

struct SqlTransaction {
    explicit SqlTransaction(sqlite3* const db) : db_{db} { ExecuteStatement("BEGIN TRANSACTION", db_); }

    ~SqlTransaction() { ExecuteStatement("END TRANSACTION", db_); }

   private:
    sqlite3* db_;
};

// TODO(Jack): Can we use concepts here to enforce some properties on Container and Binder?
template <typename Container, typename Binder>
void BatchExecuteStatement(std::string_view sql, Container const& data, Binder&& binder, sqlite3* const db) {
    for (SqlTransaction const transaction{db}; auto const& data_i : data) {
        ExecuteStatement(sql, [&](sqlite3_stmt* stmt) { binder(stmt, data_i); }, db);
    }
}

// TEST!
// WARN(Jack): Span is non-owning, therefore I think there is a real risk that in the long term we run into some
// segfaults when people use this code.
static std::span<const std::byte> SqliteBlob(sqlite3_stmt* const stmt, int const col) {
    auto const* const ptr{sqlite3_column_blob(stmt, col)};
    int const size{sqlite3_column_bytes(stmt, col)};

    if (not ptr || size <= 0) {
        return {};
    }

    return {static_cast<std::byte const*>(ptr), static_cast<size_t>(size)};
}

std::optional<std::pair<AssetId, std::string>> ReadAssetId(sqlite3* const db, AssetType const type,
                                                           size_t const index) {
    auto const binder{[type, index](sqlite3_stmt* stmt) {
        Bind(stmt, 1, ToString(type));
        Bind(stmt, 2, index);
    }};

    std::optional<std::pair<AssetId, std::string>> data;
    ExecuteQuery(db, sql_statements::assets_select, binder, [&data](sqlite3_stmt* const stmt) {
        AssetId const asset_id{sqlite3_column_int64(stmt, 0)};
        std::string const name{std::string(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 1)))};

        data = std::make_pair(asset_id, name);
    });

    return data;
}  // LCOV_EXCL_LINE

// TODO(Jack): Should we define basic structs like Hash and Name? Passing around raw strings does not scale.
std::optional<std::pair<RecordingId, std::string>> ReadRecordingId(sqlite3* const db, std::string_view name) {
    auto const binder{[name](sqlite3_stmt* stmt) { Bind(stmt, 1, name); }};

    std::optional<std::pair<RecordingId, std::string>> data;
    ExecuteQuery(db, sql_statements::recordings_select, binder, [&data](sqlite3_stmt* const stmt) {
        RecordingId const recording_id{sqlite3_column_int64(stmt, 0)};
        std::string const hash{std::string(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 1)))};

        data = std::make_pair(recording_id, hash);
    });

    return data;
}

std::optional<std::pair<RunId, std::string>> ReadRunId(sqlite3* const db, RecordingId const recording_id,
                                                       std::string_view config_hash) {
    auto const binder{[recording_id, config_hash](sqlite3_stmt* stmt) {
        Bind(stmt, 1, recording_id.value);
        Bind(stmt, 2, config_hash);
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

std::optional<std::pair<StepId, std::string>> ReadStepId(sqlite3* const db,
                                                         std::optional<RecordingId> const& recording_id,
                                                         std::optional<RunId> const& run_id, StepType type) {
    auto const binder{[recording_id, run_id, type](sqlite3_stmt* stmt) {
        recording_id ? Bind(stmt, 1, recording_id->value) : BindNull(stmt, 1);
        run_id ? Bind(stmt, 2, run_id->value) : BindNull(stmt, 2);
        Bind(stmt, 3, ToString(type));
    }};

    std::optional<std::pair<StepId, std::string>> data;
    ExecuteQuery(db, sql_statements::steps_select, binder, [&data](sqlite3_stmt* const stmt) {
        StepId const step_id{sqlite3_column_int64(stmt, 0)};
        std::string const cache_key{std::string(reinterpret_cast<char const*>(sqlite3_column_text(stmt, 1)))};

        data = std::make_pair(step_id, cache_key);
    });

    return data;
}

AssetId InsertAsset(sqlite3* const db, AssetType const type, size_t const index, std::string_view name) {
    auto const binder{[type, index, name](sqlite3_stmt* stmt) {
        Bind(stmt, 1, ToString(type));
        Bind(stmt, 2, index);
        Bind(stmt, 3, name);
    }};

    AssetId data{-1};
    ExecuteQuery(db, sql_statements::assets_insert, binder,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}  // LCOV_EXCL_LINE

RecordingId InsertRecording(sqlite3* const db, std::string_view name, std::string_view hash) {
    auto const binder{[name, hash](sqlite3_stmt* stmt) {
        Bind(stmt, 1, name);
        Bind(stmt, 2, hash);
    }};

    RecordingId data{-1};
    ExecuteQuery(db, sql_statements::recordings_insert, binder,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}  // LCOV_EXCL_LINE

RunId InsertRun(sqlite3* const db, RecordingId const recording_id, std::string_view config_hash,
                std::string_view config) {
    auto const binder{[recording_id, config_hash, config](sqlite3_stmt* stmt) {
        Bind(stmt, 1, recording_id.value);
        Bind(stmt, 2, config_hash);
        Bind(stmt, 3, config);
    }};

    RunId data{-1};
    ExecuteQuery(db, sql_statements::runs_insert, binder,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}  // LCOV_EXCL_LINE

StepId InsertStep(sqlite3* const db, std::optional<RecordingId> const& recording_id, std::optional<RunId> const& run_id,
                  StepType const type, std::string_view cache_key) {
    auto const binder{[recording_id, run_id, type, cache_key](sqlite3_stmt* stmt) {
        recording_id ? Bind(stmt, 1, recording_id->value) : BindNull(stmt, 1);
        run_id ? Bind(stmt, 2, run_id->value) : BindNull(stmt, 2);
        Bind(stmt, 3, ToString(type));
        Bind(stmt, 4, cache_key);
    }};

    StepId data{-1};
    ExecuteQuery(db, sql_statements::steps_insert, binder,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}  // LCOV_EXCL_LINE

// NOTE(Jack): This is not strictly an upsert because we actually delete the entire row and then insert it again. We do
// this to make sure that "cascade on delete" operations happen. Official upsert semantics never call delete and
// therefore cannot be used here.
StepId UpsertStep(sqlite3* const db, StepId const id, std::optional<RecordingId> const& recording_id,
                  std::optional<RunId> const& run_id, StepType const type, std::string_view cache_key) {
    auto const binder1{[id](sqlite3_stmt* stmt) { Bind(stmt, 1, id.value); }};

    StepId data{-1};
    ExecuteStatement(sql_statements::steps_delete, binder1, db);

    auto const binder2{[id, recording_id, run_id, type, cache_key](sqlite3_stmt* stmt) {
        Bind(stmt, 1, id.value);
        recording_id ? Bind(stmt, 2, recording_id->value) : BindNull(stmt, 2);
        run_id ? Bind(stmt, 3, run_id->value) : BindNull(stmt, 3);
        Bind(stmt, 4, ToString(type));
        Bind(stmt, 5, cache_key);
    }};

    // NOTE(Jack): Technically we know the id already so there is nothing that forces us to read it from the result of
    // this operation, but it is the pattern we use everywhere else and also its good to make sure what the database
    // actually processes, not just what we hope it does.
    ExecuteQuery(db, sql_statements::steps_insert_id, binder2,
                 [&data](sqlite3_stmt* const stmt) { data.value = sqlite3_column_int64(stmt, 0); });

    return data;
}  // LCOV_EXCL_LINE

class CalibrationDatabase {
   public:
    // TODO(Jack): Should we make this private and instead use a factory?
    CalibrationDatabase(fs::path const& db_path, bool const create, bool const read_only = false) {
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

    AssetId GetOrCreateAsset(AssetType const type, size_t const index, std::string_view name) {
        auto const result{ReadAssetId(db_, type, index)};
        if (result and result->second != name) {
            throw std::runtime_error(
                std::format("Asset of type '{}', index '{}' and name '{}' already exists - cannot change name '{}'.",
                            ToString(type), index, result->second, name));
        } else if (result) {
            return result->first;
        }

        return InsertAsset(db_, type, index, name);
    }

    RecordingId GetOrCreateRecording(std::string_view name, std::string_view hash) {
        auto const result{ReadRecordingId(db_, name)};
        if (result and result->second != hash) {
            throw std::runtime_error(std::format(
                "Recording '{}' with hash '{}' already exists - cannot change hash '{}'.", name, result->second, hash));
        } else if (result) {
            return result->first;
        }

        return InsertRecording(db_, name, hash);
    }

    RunId GetOrCreateRun(RecordingId const recording_id, std::string_view config) {
        // ERROR(Jack): Use a real hash functions!!!!
        std::string const config_hash{config};

        // NOTE(Jack): Unlike when the Asset/Recording logic, we add new runs if the config changes, that is why we
        // don't have an error block here.
        auto const result{ReadRunId(db_, recording_id, config_hash)};
        if (result and result->second == config_hash) {
            return result->first;
        }

        return InsertRun(db_, recording_id, config_hash, config);
    }

    // bool: was this a cache hit?
    std::pair<StepId, bool> GetOrCreateStep(std::optional<RecordingId> const& recording_id,
                                            std::optional<RunId> const& run_id, StepType const type,
                                            std::string_view cache_key) {
        auto const result{ReadStepId(db_, recording_id, run_id, type)};
        if (result and result->second == cache_key) {
            return std::make_pair(result->first, true);
        } else if (result) {
            return std::make_pair(UpsertStep(db_, result->first, recording_id, run_id, type, cache_key), false);
        }

        return std::make_pair(InsertStep(db_, recording_id, run_id, type, cache_key), false);
    }

    void ImagesInsert(StepId const step_id, AssetId const asset_id, EncodedImages const& data) {
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

    EncodedImages ImagesSelect(StepId const step_id, AssetId const asset_id) {
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

    void ExtractedTargetsInsert(StepId const step_id, StepId const source_step_id, AssetId const asset_id,
                                CameraMeasurements const& data) {
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

    CameraMeasurements ExtractedTargetsSelect(StepId const step_id, AssetId const asset_id) {
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

    void TargetInfoInsert(StepId const step_id, AssetId const asset_id, TargetInfo const& target_info) {
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

    std::optional<TargetInfo> TargetInfoSelect(StepId const step_id, AssetId const asset_id) {
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

   private:
    sqlite3* db_{nullptr};
};

}  // namespace reprojection::database