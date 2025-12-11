#include "sqlite3_helpers.hpp"

#include <iostream>

namespace reprojection::database {

[[nodiscard]] bool Sqlite3Tools::Execute(std::string const& sql_statement, sqlite3* const db, CallbackType callback,
                                         void* data_structure) {
    char* errror_msg{nullptr};
    int const code{sqlite3_exec(db, sql_statement.c_str(), callback, data_structure, &errror_msg)};

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
    sqlite3_stmt* stmt{nullptr};
    int code{sqlite3_prepare_v2(db, sql_statement.c_str(), -1, &stmt, nullptr)};
    if (code != static_cast<int>(SqliteFlag::Ok)) {
        std::cerr << "AddBlob() sqlite3_prepare_v2() failed: " << sqlite3_errmsg(db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                          // LCOV_EXCL_LINE
    }

    // Transient vs static - https://stackoverflow.com/questions/1229102/when-to-use-sqlite-transient-vs-sqlite-static
    // Note that the position is not zero indexed here! Therefore, 1 corresponds to the first (and only) binding.
    code = sqlite3_bind_int64(stmt, 1, timestamp_ns);
    if (code != static_cast<int>(SqliteFlag::Ok)) {
        std::cerr << "AddBlob() sqlite3_bind_int64() failed: " << sqlite3_errmsg(db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                          // LCOV_EXCL_LINE
    }

    code = sqlite3_bind_text(stmt, 2, sensor_name.c_str(), -1, SQLITE_STATIC);
    if (code != static_cast<int>(SqliteFlag::Ok)) {
        std::cerr << "AddBlob() sqlite3_bind_text() failed: " << sqlite3_errmsg(db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                         // LCOV_EXCL_LINE
    }

    code = sqlite3_bind_blob(stmt, 3, blob_ptr, blob_size, SQLITE_STATIC);
    if (code != static_cast<int>(SqliteFlag::Ok)) {
        std::cerr << "AddBlob() sqlite3_bind_blob() failed: " << sqlite3_errmsg(db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                         // LCOV_EXCL_LINE
    }

    code = sqlite3_step(stmt);
    if (code != static_cast<int>(SqliteFlag::Done)) {
        std::cerr << "AddBlob() sqlite3_step() failed: " << sqlite3_errmsg(db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                    // LCOV_EXCL_LINE
    }

    sqlite3_finalize(stmt);

    return true;
}

}  // namespace reprojection::database
