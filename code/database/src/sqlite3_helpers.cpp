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

}  // namespace reprojection::database
