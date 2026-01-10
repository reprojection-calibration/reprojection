#include "database/image_interface.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

#include "database/sensor_data_interface.hpp"

// cppcheck-suppress missingInclude
#include "generated/sql.hpp"
#include "sqlite3_helpers.hpp"

namespace reprojection::database {

// Adopted from https://stackoverflow.com/questions/18092240/sqlite-blob-insertion-c
void AddImage(ImageStamped const& data, std::shared_ptr<CalibrationDatabase> const database) {
    std::vector<uchar> buffer;
    if (not cv::imencode(".png", data.image, buffer)) {
        throw std::runtime_error("AddImage() cv::imencode() failed for sensor: " + data.header.sensor_name +
                                 " at timestamp_ns: " + std::to_string(data.header.timestamp_ns));
    }

    SqliteResult const result{Sqlite3Tools::AddTimeNameBlob(sql_statements::image_insert, data.header.timestamp_ns,
                                                            data.header.sensor_name, buffer.data(), std::size(buffer),
                                                            database->db)};

    if (std::holds_alternative<SqliteErrorCode>(result)) {
        throw std::runtime_error("AddImage() failed for sensor: " + data.header.sensor_name +
                                 " at timestamp_ns: " + std::to_string(data.header.timestamp_ns) +
                                 " with database error message: " + std::string(sqlite3_errmsg(database->db)));
    }
}

void AddImage(FrameHeader const& header, std::shared_ptr<CalibrationDatabase> const database) {
    SqliteResult const result{Sqlite3Tools::AddTimeNameBlob(sql_statements::image_insert, header.timestamp_ns,
                                                            header.sensor_name, nullptr, -1, database->db)};

    if (std::holds_alternative<SqliteErrorCode>(result)) {
        throw std::runtime_error("AddImage() failed for sensor: " + header.sensor_name +
                                 " at timestamp_ns: " + std::to_string(header.timestamp_ns) +
                                 " with database error message: " + std::string(sqlite3_errmsg(database->db)));
    }
}

ImageStreamer::ImageStreamer(std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name,
                             uint64_t const start_time)
    : database_{database}, statement_{database_->db, sql_statements::images_select}, sensor_name_{sensor_name} {
    try {
        Sqlite3Tools::Bind(statement_.stmt, 1, sensor_name);
        Sqlite3Tools::Bind(statement_.stmt, 2, static_cast<int64_t>(start_time));               // Warn cast!
    } catch (std::runtime_error const& e) {                                                     // LCOV_EXCL_LINE
        std::cerr << "ImageStreamer() runtime error during binding: " << e.what()               // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        throw;                                                                                  // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE
}

std::optional<ImageStamped> ImageStreamer::Next() {
    int const code{sqlite3_step(statement_.stmt)};
    if (code == static_cast<int>(SqliteFlag::Done)) {
        return std::nullopt;
    } else if (code != static_cast<int>(SqliteFlag::Row)) {
        std::cerr << "ImageStreamer::Next() sqlite3_step() failed:  "  // LCOV_EXCL_LINE
                  << sqlite3_errmsg(database_->db) << "\n";            // LCOV_EXCL_LINE
        return std::nullopt;                                           // LCOV_EXCL_LINE
    }

    uint64_t const timestamp_ns{std::stoull(reinterpret_cast<const char*>(sqlite3_column_text(statement_.stmt, 0)))};

    uchar const* const blob{static_cast<uchar const*>(sqlite3_column_blob(statement_.stmt, 1))};
    int const blob_size{sqlite3_column_bytes(statement_.stmt, 1)};
    if (not blob or blob_size <= 0) {
        std::cerr << "ImageStreamer::Next() error reading blob for timestamp: " << timestamp_ns  // LCOV_EXCL_LINE
                  << "\n";                                                                       // LCOV_EXCL_LINE
        return std::nullopt;                                                                     // LCOV_EXCL_LINE
    }

    std::vector<uchar> const buffer(blob, blob + blob_size);
    cv::Mat const image{cv::imdecode(buffer, cv::IMREAD_UNCHANGED)};
    if (image.empty()) {
        std::cerr << "ImageStreamer::Next() deserialization failed for timestamp: "  // LCOV_EXCL_LINE
                  << timestamp_ns << "\n";                                           // LCOV_EXCL_LINE
        return std::nullopt;                                                         // LCOV_EXCL_LINE
    }

    return ImageStamped{{timestamp_ns, sensor_name_}, image};
}

};  // namespace reprojection::database