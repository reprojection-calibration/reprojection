#include "database/sensor_data_interface.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

// cppcheck-suppress missingInclude
#include "generated/sql_statements.hpp"
#include "serialization.hpp"
#include "sqlite3_helpers.hpp"

namespace reprojection::database {

SqlStatement::SqlStatement(sqlite3* const db, char const* const sql) {
    if (sqlite3_prepare_v2(db, sql, -1, &stmt, nullptr) != SQLITE_OK) {
        throw std::runtime_error(sqlite3_errmsg(db));  // LCOV_EXCL_LINE
    }
}

SqlStatement::~SqlStatement() { sqlite3_finalize(stmt); }

// TODO(Jack): Make a generic pose handling utility that can be used regardless of which type of pose we are dealing
// with.
[[nodiscard]] bool AddCameraPoseData(std::string const& sensor_name, PoseData const& data,
                                     std::string const& sql_statement,
                                     std::shared_ptr<CalibrationDatabase> const database) {
    SqlStatement const statement{database->db, sql_statement.c_str()};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, static_cast<int64_t>(data.timestamp_ns));  // Warn cast!
        Sqlite3Tools::Bind(statement.stmt, 2, sensor_name.c_str());
        Sqlite3Tools::Bind(statement.stmt, 3, data.pose[0]);
        Sqlite3Tools::Bind(statement.stmt, 4, data.pose[1]);
        Sqlite3Tools::Bind(statement.stmt, 5, data.pose[2]);
        Sqlite3Tools::Bind(statement.stmt, 6, data.pose[3]);
        Sqlite3Tools::Bind(statement.stmt, 7, data.pose[4]);
        Sqlite3Tools::Bind(statement.stmt, 8, data.pose[5]);
    } catch (std::runtime_error const& e) {                                                     // LCOV_EXCL_LINE
        std::cerr << "AddInitialCameraPoseData() runtime error during binding: " << e.what()    // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                           // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        std::cerr << "AddInitialCameraPoseData() sqlite3_step() failed: " << sqlite3_errmsg(database->db) << "\n";
        return false;
    }

    return true;
}

[[nodiscard]] bool AddCameraPoseData(std::string const& sensor_name, std::set<PoseData> const& data,
                                     std::string const& sql_statement,
                                     std::shared_ptr<CalibrationDatabase> const database) {
    if (not Sqlite3Tools::Execute("BEGIN TRANSACTION", database->db)) {
        return false;
    }
    for (auto const& data_i : data) {
        if (not AddCameraPoseData(sensor_name, data_i, sql_statement, database)) {
            return false;
        }
    }
    if (not Sqlite3Tools::Execute("END TRANSACTION", database->db)) {
        return false;
    }

    return true;
}

[[nodiscard]] bool AddExtractedTargetData(std::string const& sensor_name, ExtractedTargetData const& data,
                                          std::shared_ptr<CalibrationDatabase> const database) {
    protobuf_serialization::ExtractedTargetProto const serialized{Serialize(data.target)};
    std::string buffer;
    if (not serialized.SerializeToString(&buffer)) {
        std::cerr << "ExtractedTarget serialization failed at: " << std::to_string(data.timestamp_ns)  // LCOV_EXCL_LINE
                  << "\n";                                                                             // LCOV_EXCL_LINE
        return false;                                                                                  // LCOV_EXCL_LINE
    }

    return Sqlite3Tools::AddBlob(sql_statements::extracted_target_insert, data.timestamp_ns, sensor_name,
                                 buffer.c_str(), std::size(buffer), database->db);
}

// NOTE(Jack): The logic here is very similar to the ImageStreamer class, but there are enough differences that we
// cannot easily reconcile the two and eliminate copy and past like we did for the Add* functions.
std::optional<std::set<ExtractedTargetData>> GetExtractedTargetData(
    std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name) {
    SqlStatement const statement{database->db, sql_statements::extracted_targets_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, sensor_name.c_str());
    } catch (std::runtime_error const& e) {                                                     // LCOV_EXCL_LINE
        std::cerr << "GetExtractedTargetData() runtime error during binding: " << e.what()      // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        return std::nullopt;                                                                    // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    std::set<ExtractedTargetData> data;
    while (true) {
        int const code{sqlite3_step(statement.stmt)};
        if (code == static_cast<int>(SqliteFlag::Done)) {
            break;
        } else if (code != static_cast<int>(SqliteFlag::Row)) {
            std::cerr << "GetExtractedTargetData() sqlite3_step() failed:  "  // LCOV_EXCL_LINE
                      << sqlite3_errmsg(database->db) << "\n";                // LCOV_EXCL_LINE
            return std::nullopt;                                              // LCOV_EXCL_LINE
        }

        // TODO(Jack): Should we be more defensive here and first check that column text is not returning a nullptr or
        // other bad output? Also happens like this in the image streamer.
        uint64_t const timestamp_ns{std::stoull(reinterpret_cast<const char*>(sqlite3_column_text(statement.stmt, 0)))};

        uchar const* const blob{static_cast<uchar const*>(sqlite3_column_blob(statement.stmt, 1))};
        int const blob_size{sqlite3_column_bytes(statement.stmt, 1)};
        if (not blob or blob_size <= 0) {
            std::cerr << "GetExtractedTargetData() blob empty for timestamp: " << timestamp_ns  // LCOV_EXCL_LINE
                      << "\n";                                                                  // LCOV_EXCL_LINE
            continue;                                                                           // LCOV_EXCL_LINE
        }

        std::vector<uchar> const buffer(blob, blob + blob_size);
        protobuf_serialization::ExtractedTargetProto serialized;
        serialized.ParseFromArray(buffer.data(), buffer.size());

        auto const deserialized{Deserialize(serialized)};
        if (not deserialized.has_value()) {
            std::cerr << "GetExtractedTargetData() protobuf deserialization failed for timestamp: "  // LCOV_EXCL_LINE
                      << timestamp_ns << "\n";                                                       // LCOV_EXCL_LINE
            continue;                                                                                // LCOV_EXCL_LINE
        }

        data.insert(ExtractedTargetData{timestamp_ns, deserialized.value()});
    }

    return data;
}

[[nodiscard]] bool AddImuData(std::string const& sensor_name, ImuData const& data,
                              std::shared_ptr<CalibrationDatabase> const database) {
    SqlStatement const statement{database->db, sql_statements::imu_data_insert};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, static_cast<int64_t>(data.timestamp_ns));  // Warn cast!
        Sqlite3Tools::Bind(statement.stmt, 2, sensor_name.c_str());
        Sqlite3Tools::Bind(statement.stmt, 3, data.angular_velocity[0]);
        Sqlite3Tools::Bind(statement.stmt, 4, data.angular_velocity[1]);
        Sqlite3Tools::Bind(statement.stmt, 5, data.angular_velocity[2]);
        Sqlite3Tools::Bind(statement.stmt, 6, data.linear_acceleration[0]);
        Sqlite3Tools::Bind(statement.stmt, 7, data.linear_acceleration[1]);
        Sqlite3Tools::Bind(statement.stmt, 8, data.linear_acceleration[2]);
    } catch (std::runtime_error const& e) {                                                     // LCOV_EXCL_LINE
        std::cerr << "AddImuData() runtime error during binding: " << e.what()                  // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                           // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        std::cerr << "AddImuData() sqlite3_step() failed: " << sqlite3_errmsg(database->db) << "\n";
        return false;
    }

    return true;
}

std::optional<std::set<ImuData>> GetImuData(std::shared_ptr<CalibrationDatabase const> const database,
                                            std::string const& sensor_name) {
    SqlStatement const statement{database->db, sql_statements::imu_data_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, sensor_name.c_str());
    } catch (std::runtime_error const& e) {                                                     // LCOV_EXCL_LINE
        std::cerr << "GetImuData() runtime error during binding: " << e.what()                  // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        return std::nullopt;                                                                    // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    std::set<ImuData> data;
    while (true) {
        int const code{sqlite3_step(statement.stmt)};
        if (code == static_cast<int>(SqliteFlag::Done)) {
            break;
        } else if (code != static_cast<int>(SqliteFlag::Row)) {
            std::cerr << "GetImuData() sqlite3_step() failed:  " << sqlite3_errmsg(database->db)  // LCOV_EXCL_LINE
                      << "\n";                                                                    // LCOV_EXCL_LINE
            return std::nullopt;                                                                  // LCOV_EXCL_LINE
        }

        // TODO(Jack): Should we be doing any error checking here while reading the columns?
        uint64_t const timestamp_ns{static_cast<uint64_t>(sqlite3_column_int64(statement.stmt, 0))};  // Warn cast!
        double const omega_x{sqlite3_column_double(statement.stmt, 1)};
        double const omega_y{sqlite3_column_double(statement.stmt, 2)};
        double const omega_z{sqlite3_column_double(statement.stmt, 3)};
        double const ax{sqlite3_column_double(statement.stmt, 4)};
        double const ay{sqlite3_column_double(statement.stmt, 5)};
        double const az{sqlite3_column_double(statement.stmt, 6)};

        data.insert(ImuData{timestamp_ns, {omega_x, omega_y, omega_z}, {ax, ay, az}});
    }

    return data;
}

// Adopted from https://stackoverflow.com/questions/18092240/sqlite-blob-insertion-c
bool AddImage(std::string const& sensor_name, ImageData const& data,
              std::shared_ptr<CalibrationDatabase> const database) {
    std::vector<uchar> buffer;
    if (not cv::imencode(".png", data.image, buffer)) {
        std::cerr << "Image serialization failed at: " << std::to_string(data.timestamp_ns) << "\n";  // LCOV_EXCL_LINE
        return false;                                                                                 // LCOV_EXCL_LINE
    }

    return Sqlite3Tools::AddBlob(sql_statements::image_insert, data.timestamp_ns, sensor_name, buffer.data(),
                                 std::size(buffer), database->db);
}

ImageStreamer::ImageStreamer(std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name,
                             uint64_t const start_time)
    : database_{database}, statement_{database_->db, sql_statements::images_select} {
    try {
        Sqlite3Tools::Bind(statement_.stmt, 1, sensor_name);
        Sqlite3Tools::Bind(statement_.stmt, 2, static_cast<int64_t>(start_time));               // Warn cast!
    } catch (std::runtime_error const& e) {                                                     // LCOV_EXCL_LINE
        std::cerr << "ImageStreamer() runtime error during binding: " << e.what()               // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        throw;                                                                                  // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE
}

std::optional<ImageData> ImageStreamer::Next() {
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

    return ImageData{timestamp_ns, image};
}

};  // namespace reprojection::database