#include "sqlite_wrappers.hpp"

#include "sqlite3_helpers.hpp"
#include "statement_executor.hpp"

namespace reprojection::database {

SqlStatement::SqlStatement(SqlitePtr const& db, char const* const sql) {
    if (sqlite3_prepare_v2(db.get(), sql, -1, &stmt, nullptr) != SQLITE_OK) {
        throw SqliteException(db, sql);
    }
}

SqlStatement::~SqlStatement() { sqlite3_finalize(stmt); }

SqlTransaction::SqlTransaction(SqlitePtr const& db) : db_{db} { ExecuteStatement("BEGIN TRANSACTION", db_); }

SqlTransaction::~SqlTransaction() { ExecuteStatement("END TRANSACTION", db_); }

}  // namespace reprojection::database