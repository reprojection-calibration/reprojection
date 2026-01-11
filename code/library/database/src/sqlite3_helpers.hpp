#pragma once

#include <sqlite3.h>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <variant>

#include "database/database_data_types.hpp"

namespace reprojection::database {

enum class SqliteFlag {
    Done = SQLITE_DONE,
    Ok = SQLITE_OK,
    OpenReadOnly = SQLITE_OPEN_READONLY,
    OpenReadWrite = SQLITE_OPEN_READWRITE,
    OpenCreate = SQLITE_OPEN_CREATE,
    Row = SQLITE_ROW
};

// TODO(Jack): Is there anyway that we can use official sqlite error codes like the Sqlite flags above? See note above
// SqliteResult.
enum class SqliteErrorCode { FailedBinding, FailedStep };

std::string ToString(SqliteErrorCode const enumerator);

// TODO(Jack): Naming! "result" is way too generic!
// WARN(Jack): We are abusing the SqliteFlag here because I am not really sure that they are the positive result of the
// operation, I simply saw it was "ok" and saw a flag that said Ok and was happy with it. This is different than there
// usage in the sensor interface code (ex. code == static_cast<int>(SqliteFlag::Done)) where they correspond to actually
// codes returned by the sqlite functions.
using SqliteResult = std::variant<SqliteFlag, SqliteErrorCode>;

struct Sqlite3Tools {
    using CallbackType = int (*)(void*, int, char**, char**);

    [[nodiscard]] static bool Execute(std::string const& sql_statement, sqlite3* const db);

    // TODO(Jack): Test!
    // TODO(Jack): My original intention was a AddBlob function where the blob itself was the only parameter, and the
    // rest of sql statement was formed dynamically before calling this function. That would allow us to use this add
    // blob function for any table with any possible layout, because only the blob component was being added in this
    // method. However when we started to put the sql definitions in .sql files we need to make the statements as
    // generic as possible which meant using place holders (i.e. ? etc.). This means that in the AddBlob method we
    // actually need to fill out the other attributes too, for example the timestamp_ns and sensor_name. This fact
    // eliminated and generic component of the function and it now means that this method can only be used for inserting
    // into a table that holds blobs indexed by timestamp and the sensor name. This is no strictly a problem, but the
    // fact that this method now takes six arguments tells you that maybe we are missing an abstraction :)
    [[nodiscard]] static SqliteResult AddTimeNameBlob(std::string const& sql_statement, uint64_t const timestamp_ns,
                                                      std::string const& sensor_name, void const* const blob_ptr,
                                                      int const blob_size, sqlite3* const db);

    [[nodiscard]] static SqliteResult AddTimeNameTypeBlob(std::string const& sql_statement, uint64_t const timestamp_ns,
                                                          PoseType const type, std::string const& sensor_name,
                                                          void const* const blob_ptr, int const blob_size,
                                                          sqlite3* const db);

    [[nodiscard]] static SqliteResult AddBlob(std::string const& sql_statement, uint64_t const timestamp_ns,
                                              std::string const& sensor_name, void const* const blob_ptr,
                                              int const blob_size, sqlite3* const db,
                                              std::optional<PoseType> const& type = std::nullopt);

    static void Bind(sqlite3_stmt* const stmt, int const index, std::string const& value) {
        if (sqlite3_bind_text(stmt, index, value.c_str(), -1, SQLITE_TRANSIENT) != static_cast<int>(SqliteFlag::Ok)) {
            throw std::runtime_error("sqlite3_bind_text() failed");
        }
    }

    // WARN(Jack): We store our timestamps in uint64_t but sqlite only takes signed int64. How should we fix this?
    static void Bind(sqlite3_stmt* const stmt, int const index, int64_t const value) {
        if (sqlite3_bind_int64(stmt, index, value) != static_cast<int>(SqliteFlag::Ok)) {
            throw std::runtime_error("sqlite3_bind_int64() failed");
        }
    }

    static void Bind(sqlite3_stmt* const stmt, int const index, double const value) {
        if (sqlite3_bind_double(stmt, index, value) != static_cast<int>(SqliteFlag::Ok)) {
            throw std::runtime_error("sqlite3_bind_double() failed");
        }
    }

    static void BindBlob(sqlite3_stmt* const stmt, int const index, void const* const blob_ptr, int const blob_size) {
        if (sqlite3_bind_blob(stmt, index, blob_ptr, blob_size, SQLITE_STATIC) != static_cast<int>(SqliteFlag::Ok)) {
            throw std::runtime_error("sqlite3_bind_blob() failed");
        }
    }
};



std::string ErrorMessage(std::string const& function_name, std::string const& sensor_name, uint64_t const timestamp_ns,
                         SqliteErrorCode const error_code, std::string const& db_error_message);

}  // namespace reprojection::database
