#include "database/image_interface.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

#include "database/sensor_data_interface_adders.hpp"

// cppcheck-suppress missingInclude
#include "generated/sql.hpp"

#include "sqlite3_helpers.hpp"

namespace reprojection::database {

// Adopted from https://stackoverflow.com/questions/18092240/sqlite-blob-insertion-c
void AddImage(CameraImage const& data, std::string_view sensor_name,
              std::shared_ptr<CalibrationDatabase> const database) {
    auto const& [timestamp_ns, image]{data};

    std::vector<uchar> buffer;
    if (not cv::imencode(".png", data.second, buffer)) {
        throw std::runtime_error(
            "AddImage() cv::imencode() failed for sensor: " + std::string(sensor_name) +  // LCOV_EXCL_LINE
            " at timestamp_ns: " + std::to_string(timestamp_ns));                         // LCOV_EXCL_LINE
    }

    SqliteResult const result{Sqlite3Tools::AddBlob(sql_statements::image_insert, DataKey{sensor_name, timestamp_ns},
                                                    buffer.data(), std::size(buffer), database->db)};

    if (std::holds_alternative<SqliteErrorCode>(result)) {
        throw std::runtime_error(ErrorMessage("AddImage()", "N/A", sensor_name, timestamp_ns,
                                              std::get<SqliteErrorCode>(result),
                                              std::string(sqlite3_errmsg(database->db))));
    }
}

void AddImage(uint64_t const timestamp_ns, std::string_view sensor_name,
              std::shared_ptr<CalibrationDatabase> const database) {
    SqliteResult const result{Sqlite3Tools::AddBlob(sql_statements::image_insert, DataKey{sensor_name, timestamp_ns},
                                                    nullptr, -1, database->db)};

    if (std::holds_alternative<SqliteErrorCode>(result)) {
        throw std::runtime_error(ErrorMessage("AddImage()", "N/A", sensor_name, timestamp_ns,
                                              std::get<SqliteErrorCode>(result),
                                              std::string(sqlite3_errmsg(database->db))));
    }
}

ImageStreamer::ImageStreamer(std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name,
                             uint64_t const start_time)
    : database_{database}, statement_{database_->db, sql_statements::images_select}, sensor_name_{sensor_name} {
    try {
        Sqlite3Tools::Bind(statement_.stmt, 1, sensor_name);
        Sqlite3Tools::Bind(statement_.stmt, 2, static_cast<int64_t>(start_time));  // Warn cast!
    } catch (std::runtime_error const& e) {                                        // LCOV_EXCL_LINE
        std::throw_with_nested(std::runtime_error(                                 // LCOV_EXCL_LINE
            ErrorMessage("ImageStreamer::ImageStreamer()", "N/A", sensor_name_,    // LCOV_EXCL_LINE
                         start_time, SqliteErrorCode::FailedBinding,               // LCOV_EXCL_LINE
                         std::string(sqlite3_errmsg(database_->db)))));            // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE
}

std::optional<CameraImage> ImageStreamer::Next() {
    int const code{sqlite3_step(statement_.stmt)};
    if (code == static_cast<int>(SqliteFlag::Done)) {
        return std::nullopt;
    } else if (code != static_cast<int>(SqliteFlag::Row)) {
        std::cerr << ErrorMessage("ImageStreamer::Next()", "N/A", sensor_name_, 0,  // LCOV_EXCL_LINE
                                  SqliteErrorCode::FailedStep,                      // LCOV_EXCL_LINE
                                  std::string(sqlite3_errmsg(database_->db)))       // LCOV_EXCL_LINE
                  << "\n";                                                          // LCOV_EXCL_LINE
        return std::nullopt;                                                        // LCOV_EXCL_LINE
    }

    uint64_t const timestamp_ns{std::stoull(reinterpret_cast<const char*>(sqlite3_column_text(statement_.stmt, 0)))};

    uchar const* const blob{static_cast<uchar const*>(sqlite3_column_blob(statement_.stmt, 1))};
    int const blob_size{sqlite3_column_bytes(statement_.stmt, 1)};
    if (not blob or blob_size <= 0) {
        std::cerr << "ImageStreamer::Next() blob reading failed for sensor: " + sensor_name_ +  // LCOV_EXCL_LINE
                         " at timestamp_ns: " + std::to_string(timestamp_ns)                    // LCOV_EXCL_LINE
                  << "\n";                                                                      // LCOV_EXCL_LINE
        return std::nullopt;                                                                    // LCOV_EXCL_LINE
    }

    std::vector<uchar> const buffer(blob, blob + blob_size);
    cv::Mat const image{cv::imdecode(buffer, cv::IMREAD_UNCHANGED)};
    if (image.empty()) {
        std::cerr << "ImageStreamer::Next() cv::imdecode() failed for sensor: " + sensor_name_ +  // LCOV_EXCL_LINE
                         " at timestamp_ns: " + std::to_string(timestamp_ns)                      // LCOV_EXCL_LINE
                  << "\n";                                                                        // LCOV_EXCL_LINE
        return std::nullopt;                                                                      // LCOV_EXCL_LINE
    }

    return CameraImage{timestamp_ns, image};
}

};  // namespace reprojection::database