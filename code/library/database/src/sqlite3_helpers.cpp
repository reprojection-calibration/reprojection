#include "sqlite3_helpers.hpp"

#include <iostream>

// TODO(Jack): Is this not kind of a circular include?
#include "database/sensor_data_interface.hpp"
#include "database/sql_statement_wrapper.hpp"

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
    } catch (std::runtime_error const& e) {                                           // LCOV_EXCL_LINE
        std::cerr << "AddBlob() runtime error during binding: " << e.what()           // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                 // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        std::cerr << "AddBlob() sqlite3_step() failed: " << sqlite3_errmsg(db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                    // LCOV_EXCL_LINE
    }

    return true;
}

}  // namespace reprojection::database
