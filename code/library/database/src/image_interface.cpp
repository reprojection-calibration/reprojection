#include "database/image_interface.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

#include "database/database_write.hpp"

// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "sqlite3_helpers.hpp"
#include "statement_executor.hpp"

namespace reprojection::database {

// Adopted from https://stackoverflow.com/questions/18092240/sqlite-blob-insertion-c
void AddImage(Image const& data, std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[&data, sensor_name](sqlite3_stmt* const stmt) {
        auto const& [timestamp_ns, image]{data};

        std::vector<uchar> buffer;
        if (not cv::imencode(".png", data.second, buffer)) {
            throw std::runtime_error("cv::imencode() failed for " + std::string(sensor_name));  // LCOV_EXCL_LINE
        }

        Sqlite3Tools::Bind(stmt, 1, std::string(sensor_name));
        Sqlite3Tools::Bind(stmt, 2, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
        Sqlite3Tools::BindBlob(stmt, 3, std::as_bytes(std::span{buffer}));
    }};

    ExecuteStatement(sql_statements::image_insert, binder, db);
}

void AddImage(uint64_t const timestamp_ns, std::string_view sensor_name, SqlitePtr const db) {
    auto const binder{[timestamp_ns, sensor_name](sqlite3_stmt* const stmt) {
        Sqlite3Tools::Bind(stmt, 1, std::string(sensor_name));
        Sqlite3Tools::Bind(stmt, 2, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
    }};

    ExecuteStatement(sql_statements::image_insert, binder, db);
}

};  // namespace reprojection::database