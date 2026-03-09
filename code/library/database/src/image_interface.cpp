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
void AddImage(CameraImage const& data, std::string_view sensor_name,
              std::shared_ptr<CalibrationDatabase> const database) {
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

    ExecuteStatement(sql_statements::image_insert, binder, database->db);
}

void AddImage(uint64_t const timestamp_ns, std::string_view sensor_name,
              std::shared_ptr<CalibrationDatabase> const database) {
    auto const binder{[timestamp_ns, sensor_name](sqlite3_stmt* const stmt) {
        Sqlite3Tools::Bind(stmt, 1, std::string(sensor_name));
        Sqlite3Tools::Bind(stmt, 2, static_cast<int64_t>(timestamp_ns));  // Possible dangerous cast!
    }};

    ExecuteStatement(sql_statements::image_insert, binder, database->db);
}

ImageStreamer::ImageStreamer(std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name,
                             uint64_t const start_time)
    : database_{database}, statement_{database_->db, sql_statements::images_select}, sensor_name_{sensor_name} {
    try {
        Sqlite3Tools::Bind(statement_.stmt, 1, sensor_name);
        Sqlite3Tools::Bind(statement_.stmt, 2, static_cast<int64_t>(start_time));  // Warn cast!
    } catch (...) {                                                                // LCOV_EXCL_LINE
        throw SqliteException(database->db, "");                                   // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE
}

std::optional<CameraImage> ImageStreamer::Next() const {
    if (not Sqlite3Tools::StepRow(statement_.stmt)) {
        return std::nullopt;
    }

    uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(statement_.stmt, 0))};
    auto const blob{Sqlite3Tools::SqliteBlob(statement_.stmt, 1)};
    if (std::empty(blob)) {
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    std::span<uchar const> buffer{reinterpret_cast<uchar const*>(blob.data()), blob.size()};
    cv::Mat const image{cv::imdecode(std::vector<uchar>(std::cbegin(buffer), std::cend(buffer)), cv::IMREAD_UNCHANGED)};
    if (image.empty()) {
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    return CameraImage{timestamp_ns, image};
}

};  // namespace reprojection::database