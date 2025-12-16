#include "database/sqlite_wrappers.hpp"

#include <stdexcept>

#include "sqlite3_helpers.hpp"

namespace reprojection::database {

SqlStatement::SqlStatement(sqlite3* const db, char const* const sql) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error(sqlite3_errmsg(db));  // LCOV_EXCL_LINE
    }
}

SqlStatement::~SqlStatement() { sqlite3_finalize(stmt); }

SqlTransaction::SqlTransaction(sqlite3* const db) : db_{db} {
    if (not Sqlite3Tools::Execute("BEGIN TRANSACTION", db_)) {
        throw std::runtime_error(sqlite3_errmsg(db));  // LCOV_EXCL_LINE
    }
}

SqlTransaction::~SqlTransaction() { static_cast<void>(Sqlite3Tools::Execute("END TRANSACTION", db_)); }

}  // namespace reprojection::database