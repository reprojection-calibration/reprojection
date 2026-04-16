#pragma once

#include <sqlite3.h>

#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "database/calibration_database.hpp"

#include "enums.hpp"
#include "types.hpp"

namespace reprojection::database {

struct Sqlite3Tools {
    static void Bind(sqlite3_stmt* const stmt, int const index, std::string_view value) {
        if (sqlite3_bind_text(stmt, index, std::string(value).c_str(), -1, SQLITE_TRANSIENT) !=
            static_cast<int>(SqliteFlag::Ok)) {
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

    // NOTE(Jack): We use SQLITE_TRANSIENT here because the serialized buffers in the lambdas disappear when the lambda
    // is finished. Therefore, we want sql to make its own copy of the buffer when we call bind (i.e. SQLITE_TRANSIENT),
    // so that way the external lifetime management can be disregarded.
    static void BindBlob(sqlite3_stmt* const stmt, int const index, std::span<std::byte const> const& blob) {
        if (sqlite3_bind_blob(stmt, index, std::data(blob), std::size(blob), SQLITE_TRANSIENT) !=
            static_cast<int>(SqliteFlag::Ok)) {
            throw std::runtime_error("sqlite3_bind_blob() failed");
        }
    }

    // TEST!
    static bool StepRow(sqlite3_stmt* const stmt) {
        int const code{sqlite3_step(stmt)};

        if (code == SQLITE_ROW) {
            return true;
        } else if (code == SQLITE_DONE) {
            return false;
        } else {
            throw std::runtime_error("SQLite step failed");
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
};

class SqliteException : public std::runtime_error {
   public:
    SqliteException(SqlitePtr const db, std::string_view sql) : std::runtime_error(FormatMessage(db, sql)) {}

   private:
    static std::string FormatMessage(SqlitePtr const db, std::string_view sql) {
        return "\n[SQLite Exception]\n"
               "----------------------------------------\n"
               "SQL Query:\n" +
               Indent(sql) + "\n\n" + "Error Code : " + std::to_string(sqlite3_errcode(db.get())) + "\n" +
               "Error Msg  : " + std::string(sqlite3_errmsg(db.get())) + "\n" +
               "----------------------------------------";
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
    }
};

}  // namespace reprojection::database
