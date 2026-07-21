
#include "sqlite_helpers.hpp"

namespace reprojection::database {

SqlStatement::SqlStatement(sqlite3* const db, char const* const sql) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt_, nullptr) != SQLITE_OK) {
        throw SqliteException(db, sql);
    }
}

SqlStatement::~SqlStatement() { sqlite3_finalize(stmt_); };

SqlTransaction::SqlTransaction(sqlite3* const db) : db_{db} { ExecuteStatement("BEGIN TRANSACTION", db_); }

SqlTransaction::~SqlTransaction() { ExecuteStatement("END TRANSACTION", db_); }

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

void BindNull(sqlite3_stmt* const stmt, int const index) {
    if (sqlite3_bind_null(stmt, index) != SQLITE_OK) {
        throw SqliteException(stmt);
    }
}

// NOTE(Jack): We use SQLITE_TRANSIENT here because the serialized buffers in the lambdas disappear when the lambda
// is finished. Therefore, we want sql to make its own copy of the buffer when we call bind (i.e. SQLITE_TRANSIENT),
// so that way the external lifetime management can be disregarded.
void BindBlob(sqlite3_stmt* const stmt, int const index, std::span<std::byte const> const& blob) {
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

// TODO(Jack): Test!
// WARN(Jack): Span is non-owning, therefore I think there is a real risk that in the long term we run into some
// segfaults when people use this code.
std::span<const std::byte> SqliteBlob(sqlite3_stmt* const stmt, int const col) {
    auto const* const ptr{sqlite3_column_blob(stmt, col)};
    int const size{sqlite3_column_bytes(stmt, col)};

    if (not ptr || size <= 0) {
        return {};
    }

    return {static_cast<std::byte const*>(ptr), static_cast<size_t>(size)};
}

void ExecuteStatement(std::string_view sql, sqlite3* const db) {
    // Passes an empty lambda which is a no-op.
    ExecuteStatement(sql, [](sqlite3_stmt*) {}, db);
}

}  // namespace reprojection::database