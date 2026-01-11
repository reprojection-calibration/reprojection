#include "sqlite3_helpers.hpp"

#include <iostream>

// TODO(Jack): Is this not kind of a circular include?
#include "database/sensor_data_interface.hpp"
#include "database/sqlite_wrappers.hpp"

namespace reprojection::database {

std::string ToString(SqliteErrorCode const enumerator) {
    if (enumerator == SqliteErrorCode::FailedBinding) {
        return "SqliteErrorCode::FailedBinding";

    } else if (enumerator == SqliteErrorCode::FailedStep) {
        return "SqliteErrorCode::FailedStep";
    } else {
        throw std::runtime_error(
            "Unknown SqliteErrorCode - this is a library implementation error!");  // LCOV_EXCL_LINE
    }
}

[[nodiscard]] bool Sqlite3Tools::Execute(std::string const& sql_statement, sqlite3* const db) {
    char* error_msg{nullptr};
    int const code{sqlite3_exec(db, sql_statement.c_str(), nullptr, nullptr, &error_msg)};

    if (code != static_cast<int>(SqliteFlag::Ok)) {
        std::cerr << "SQL error: " << error_msg << std::endl;
        sqlite3_free(error_msg);  // WARN(Jack): Violating RAII here! Should wrap errror_msg with class?

        return false;
    }

    return true;
}

[[nodiscard]] SqliteResult Sqlite3Tools::AddTimeNameBlob(std::string const& sql_statement, uint64_t const timestamp_ns,
                                                         std::string const& sensor_name, void const* const blob_ptr,
                                                         int const blob_size, sqlite3* const db) {
    return AddBlob(sql_statement, timestamp_ns, sensor_name, blob_ptr, blob_size, db);
}

[[nodiscard]] SqliteResult Sqlite3Tools::AddTimeNameTypeBlob(std::string const& sql_statement,
                                                             uint64_t const timestamp_ns, PoseType const type,
                                                             std::string const& sensor_name, void const* const blob_ptr,
                                                             int const blob_size, sqlite3* const db) {
    return AddBlob(sql_statement, timestamp_ns, sensor_name, blob_ptr, blob_size, db, type);
}

[[nodiscard]] SqliteResult Sqlite3Tools::AddBlob(std::string const& sql_statement, uint64_t const timestamp_ns,
                                                 std::string const& sensor_name, void const* const blob_ptr,
                                                 int const blob_size, sqlite3* const db,
                                                 std::optional<PoseType> const& type) {
    SqlStatement const statement{db, sql_statement.c_str()};

    try {
        Bind(statement.stmt, 1, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
        Bind(statement.stmt, 2, sensor_name.c_str());

        if (type.has_value()) {
            Bind(statement.stmt, 3, ToString(type.value()));
            BindBlob(statement.stmt, 4, blob_ptr, blob_size);
        } else {
            BindBlob(statement.stmt, 3, blob_ptr, blob_size);
        }
    } catch (std::runtime_error const& e) {
        return SqliteErrorCode::FailedBinding;
    }

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        return SqliteErrorCode::FailedStep;
    }

    return SqliteFlag::Ok;
}

}  // namespace reprojection::database
