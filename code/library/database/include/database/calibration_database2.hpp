#pragma once

#include <sqlite3.h>

#include <filesystem>
#include <format>
#include <optional>

#include "generated/sql2.hpp"

namespace reprojection::database {

namespace fs = std::filesystem;

// NOTE(Jack): sqlite only supports int64_t as the int type, so instead of static casting later we just set the ids
// directly as this type.
struct AssetId {
    int64_t value;
};

enum class AssetType { Camera, Imu, Target };

std::string ToString(AssetType const data) {
    if (data == AssetType::Camera) {
        return "camera";
    } else if (data == AssetType::Imu) {
        return "imu";
    } else if (data == AssetType::Target) {
        return "target";
    } else {
        throw std::runtime_error("Unknown AssetType - Library implementation error.");
    }
}

class SqliteException : public std::runtime_error {  // LCOV_EXCL_LINE
   public:
    SqliteException(sqlite3* const db, std::string_view sql) : std::runtime_error(FormatMessage(db, sql)) {}

    SqliteException(sqlite3* const db, sqlite3_stmt* const stmt) : std::runtime_error(FormatMessage(db, stmt)) {}

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
            // TODO(Jack): We should really query the db for the real error message!
            throw std::runtime_error("SqlStatement() constructor failed.");
        }
    }

    ~SqlStatement() { sqlite3_finalize(stmt_); }

    sqlite3_stmt* stmt_{nullptr};
};

void Bind(sqlite3_stmt* const stmt, int const index, std::string_view value) {
    if (sqlite3_bind_text(stmt, index, std::string(value).c_str(), -1, SQLITE_TRANSIENT) != SQLITE_OK) {
        // TODO(Jack): We should really query the db for the real error message!
        throw std::runtime_error("sqlite3_bind_text() failed");
    }
}

void Bind(sqlite3_stmt* const stmt, int const index, int64_t const value) {
    if (sqlite3_bind_int64(stmt, index, value) != SQLITE_OK) {
        // TODO(Jack): We should really query the db for the real error message!
        throw std::runtime_error("sqlite3_bind_int64() failed");
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

class CalibrationDatabase {
   public:
    // TODO(Jack): Should we make this private and instead use a factory?
    CalibrationDatabase(fs::path const& db_path, bool const create, bool const read_only = false);

    AssetId GetOrCreateAsset(AssetType const type, size_t const index, std::string_view name) {
        // TODO(Jack): Would it be prudent to also read the asset by the name instead and check there there is no
        // type/index with a duplicated name?
        auto const result{ReadAssetId(db_, type, index)};
        if (result and result->second == name) {
            throw std::runtime_error(
                std::format("Asset of type {}, index {} and name {} already exists with a different name {}.",
                            ToString(type), index, name, result->second));
        } else if (result) {
            return result->first;
        }
    }

   private:
    sqlite3* db_;
};

}  // namespace reprojection::database