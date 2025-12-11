#include "sqlite3_helpers.hpp"

#include <iostream>

namespace reprojection::database {

[[nodiscard]] bool Sqlite3Tools::Execute(std::string const& sql_statement, sqlite3* const db) {
    char* errror_msg{nullptr};
    int const code{sqlite3_exec(db, sql_statement.c_str(), nullptr, nullptr, &errror_msg)};

    if (code != static_cast<int>(SqliteFlag::Ok)) {
        std::cerr << "SQL error: " << errror_msg << std::endl;
        sqlite3_free(errror_msg);  // WARN(Jack): Violating RAII here! Should wrap errror_msg with class.

        return false;
    }

    return true;
}

[[nodiscard]] bool Sqlite3Tools::AddBlob(std::string const& sql_statement, uint64_t const timestamp_ns,
                                         std::string const& sensor_name, void const* const blob_ptr,
                                         int const blob_size, sqlite3* const db) {
    SqlStatement const statement{db, sql_statement.c_str()};

    try {
        Bind(statement.stmt, 1, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
        Bind(statement.stmt, 2, sensor_name.c_str());
        BindBlob(statement.stmt, 3, blob_ptr, blob_size);
    } catch (std::runtime_error const& e) {
        std::cerr << "AddBlob() runtime error during binding: " << e.what()           // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(db) << "\n";  // LCOV_EXCL_LINE
        return false;
    }

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        std::cerr << "AddBlob() sqlite3_step() failed: " << sqlite3_errmsg(db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                    // LCOV_EXCL_LINE
    }

    return true;
}

SqlStatement::SqlStatement(sqlite3* const db, char const* const sql) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error(sqlite3_errmsg(db));
    }
}

SqlStatement::~SqlStatement() { sqlite3_finalize(stmt); };

}  // namespace reprojection::database
