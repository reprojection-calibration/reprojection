#include "sqlite3_helpers.hpp"

#include <iostream>

// TODO(Jack): Is this not kind of a circular include?
#include "database/sensor_data_interface_adders.hpp"
#include "database/sqlite_wrappers.hpp"

namespace reprojection::database {

std::string ToString(SqliteErrorCode const enumerator) {
    if (enumerator == SqliteErrorCode::FailedBinding) {
        return "SqliteErrorCode::FailedBinding";  // LCOV_EXCL_LINE
    } else if (enumerator == SqliteErrorCode::FailedStep) {
        return "SqliteErrorCode::FailedStep";
    } else {
        throw std::runtime_error(
            "Unknown SqliteErrorCode - this is a library implementation error!");  // LCOV_EXCL_LINE
    }
}

// TODO(Jack): Update to be consistent with error handling strategy of the Blob helpers!
[[nodiscard]] bool Sqlite3Tools::Execute(std::string_view sql_statement, sqlite3* const db) {
    // WARN(Jack): String view .c_str() hack taken from
    // https://stackoverflow.com/questions/48081436/how-you-convert-a-stdstring-view-to-a-const-char
    char* error_msg{nullptr};
    int const code{sqlite3_exec(db, std::string(sql_statement).c_str(), nullptr, nullptr, &error_msg)};

    if (code != static_cast<int>(SqliteFlag::Ok)) {
        std::cerr << "SQL error: " << error_msg << std::endl;
        sqlite3_free(error_msg);  // WARN(Jack): Violating RAII here! Should wrap errror_msg with class?

        return false;
    }

    return true;
}

[[nodiscard]] SqliteResult Sqlite3Tools::AddTimeNameBlob(std::string const& sql_statement, std::string_view sensor_name,
                                                         uint64_t const timestamp_ns, void const* const blob_ptr,
                                                         int const blob_size, sqlite3* const db) {
    return AddBlob(sql_statement, timestamp_ns, sensor_name, blob_ptr, blob_size, db);
}

// TODO(Jack): Convert step_name to be an enum!
[[nodiscard]] SqliteResult Sqlite3Tools::AddStepTimeNameBlob(std::string const& sql_statement,
                                                             std::string_view step_name, std::string_view sensor_name,
                                                             uint64_t const timestamp_ns, void const* const blob_ptr,
                                                             int const blob_size, sqlite3* const db) {
    return AddBlob(sql_statement, timestamp_ns, sensor_name, blob_ptr, blob_size, db, step_name);
}

[[nodiscard]] SqliteResult Sqlite3Tools::AddBlob(std::string const& sql_statement, uint64_t const timestamp_ns,
                                                 std::string_view sensor_name, void const* const blob_ptr,
                                                 int const blob_size, sqlite3* const db,
                                                 std::optional<std::string_view> const& step_name) {
    SqlStatement const statement{db, sql_statement.c_str()};

    // TODO(Jack): Do not change argument order here - if there is a step_name that needs to come first!
    try {
        if (step_name.has_value()) {
            Bind(statement.stmt, 1, step_name.value());
            Bind(statement.stmt, 2, std::string(sensor_name).c_str());
            Bind(statement.stmt, 3, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
            BindBlob(statement.stmt, 4, blob_ptr, blob_size);
        } else {
            Bind(statement.stmt, 1, std::string(sensor_name).c_str());
            Bind(statement.stmt, 2, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
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

// TODO(Jack): Add step_name?
std::string ErrorMessage(std::string const& function_name, std::string_view sensor_name, uint64_t const timestamp_ns,
                         SqliteErrorCode const error_code, std::string const& db_error_message) {
    return function_name + " failed at timestamp_ns: " + std::to_string(timestamp_ns) +
           " for sensor: " + std::string(sensor_name) + " with: " + ToString(error_code) +
           " and database error message: " + db_error_message;
}

}  // namespace reprojection::database
