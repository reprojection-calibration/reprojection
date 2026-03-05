#include "sqlite_wrappers.hpp"

#include <stdexcept>

#include "sqlite3_helpers.hpp"
#include "statement_executor.hpp"

namespace reprojection::database {

SqlStatement::SqlStatement(sqlite3* const db, char const* const sql) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error(sqlite3_errmsg(db));
    }
}

SqlStatement::~SqlStatement() { sqlite3_finalize(stmt); }

SqlTransaction::SqlTransaction(sqlite3* const db) : db_{db} { ExecuteStatement("BEGIN TRANSACTION", db_); }

SqlTransaction::~SqlTransaction() { ExecuteStatement("END TRANSACTION", db_); }

}  // namespace reprojection::database