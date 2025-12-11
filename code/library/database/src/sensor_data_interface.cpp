#include "database/sensor_data_interface.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

#include "generated/sql_statements.hpp"
#include "serialization.hpp"
#include "sql.hpp"
#include "sqlite3_helpers.hpp"

namespace reprojection::database {

// TODO(Jack): There is really a lot of copy and paste between here and the image version!
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
    } catch (std::runtime_error const& e) {
        std::cerr << "GetExtractedTargetData() runtime error during binding: " << e.what()      // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        return std::nullopt;
    }

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
            std::cerr << "GetExtractedTargetData() deserialization failed for timestamp: "  // LCOV_EXCL_LINE
                      << timestamp_ns << "\n";                                              // LCOV_EXCL_LINE
            continue;                                                                       // LCOV_EXCL_LINE
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
    } catch (std::runtime_error const& e) {
        std::cerr << "AddImuData() runtime error during binding: " << e.what()                  // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        return false;
    }

    if (sqlite3_step(statement.stmt) != static_cast<int>(SqliteFlag::Done)) {
        std::cerr << "AddImuData() sqlite3_step() failed: " << sqlite3_errmsg(database->db) << "\n";
        return false;
    }

    return true;
}

std::optional<std::set<ImuData>> GetImuData(std::shared_ptr<CalibrationDatabase const> const database,
                                            std::string const& sensor_name) {
    std::string const select_imu_sensor_data_sql{SelectImuSensorDataSql(sensor_name)};
    // This callback will be executed on every row returned by the select_imu_sensor_data_sql query.
    auto callback = [](void* data, int, char** argv, char**) -> int {
        auto* const set = static_cast<std::set<ImuData>*>(data);
        set->insert(ImuData{std::stoull(argv[0]),
                            {std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3])},
                            {std::stod(argv[4]), std::stod(argv[5]), std::stod(argv[6])}});
        return 0;
    };

    std::set<ImuData> data;
    bool const success{Sqlite3Tools::Execute(select_imu_sensor_data_sql, database->db, callback, &data)};
    if (not success) {
        // NOTE(Jack): I do not know under what conditions we will ever hit this failure condition, and cannot engineer
        // a test to cover this branch, so we have to supress the code coverage requirement. This is a sign maybe we are
        // missing the point with our design of Execute.
        return std::nullopt;  // LCOV_EXCL_LINE
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
    : database_{database}, statement_{database_->db, ImageStreamerSql(sensor_name, start_time).c_str()} {}

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