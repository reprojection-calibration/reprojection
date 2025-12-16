#include "database/sql_statement_wrapper.hpp"

#include <stdexcept>

namespace reprojection::database {

SqlStatement::SqlStatement(sqlite3* const db, char const* const sql) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error(sqlite3_errmsg(db));  // LCOV_EXCL_LINE
    }
}

SqlStatement::~SqlStatement() { sqlite3_finalize(stmt); }

}  // namespace reprojection::database