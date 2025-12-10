#include "database/sensor_data_interface.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

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
        return false;  // LCOV_EXCL_LINE
    }

    std::string const insert_extracted_target_sql{InsertExtractedTargetSql(sensor_name, data.timestamp_ns)};

    return Sqlite3Tools::AddBlob(insert_extracted_target_sql, buffer.c_str(), std::size(buffer), database->db);
}

std::optional<std::set<ExtractedTargetData>> GetExtractedTargetData(
    std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name) {
    std::string const select_extracted_target_data_sql{SelectExtractedTargetDataSql(sensor_name)};
    sqlite3_stmt* stmt_{nullptr};
    int code{sqlite3_prepare_v2(database->db, select_extracted_target_data_sql.c_str(), -1, &stmt_, nullptr)};
    if (code != static_cast<int>(SqliteFlag::Ok)) {
        throw std::runtime_error("Get extracted target sqlite3_prepare_v2() failed: " +  // LCOV_EXCL_LINE
                                 std::string{sqlite3_errmsg(database->db)});             // LCOV_EXCL_LINE
    }

    std::set<ExtractedTargetData> data;
    while (true) {
        code = sqlite3_step(stmt_);
        if (code == static_cast<int>(SqliteFlag::Done)) {
            break;
        } else if (code != static_cast<int>(SqliteFlag::Row)) {
            std::cerr << "Error stepping: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
            return std::nullopt;                                                      // LCOV_EXCL_LINE
        }

        uint64_t const timestamp_ns{std::stoull(reinterpret_cast<const char*>(sqlite3_column_text(stmt_, 0)))};

        uchar const* const blob{static_cast<uchar const*>(sqlite3_column_blob(stmt_, 1))};
        int const blob_size{sqlite3_column_bytes(stmt_, 1)};
        if (not blob or blob_size <= 0) {
            // TODO(Jack): This seems to be an aggressive error handling strategy, that if there is problem with any
            // single blob we kill the entire process. Is there any "better" way to do this or any real reason why this
            // is as it is bad?
            std::cerr << "Empty blob for timestamp: " << timestamp_ns << "\n";  // LCOV_EXCL_LINE
            return std::nullopt;                                                // LCOV_EXCL_LINE
        }

        std::vector<uchar> const buffer(blob, blob + blob_size);
        protobuf_serialization::ExtractedTargetProto serialized;
        serialized.ParseFromArray(buffer.data(), buffer.size());

        auto const deserialized{Deserialize(serialized)};
        if (not deserialized.has_value()) {
            // TODO(Jack): Very aggressive error handling like above!
            std::cerr << "Could not deserialize protobuf for timestamp: " << timestamp_ns << "\n";  // LCOV_EXCL_LINE
            return std::nullopt;                                                                    // LCOV_EXCL_LINE
        }

        data.insert(ExtractedTargetData{timestamp_ns, deserialized.value()});
    }

    return data;
}

[[nodiscard]] bool AddImuData(std::string const& sensor_name, ImuData const& data,
                              std::shared_ptr<CalibrationDatabase> const database) {
    std::string const insert_imu_data_sql{
        InsertImuDataSql(data.timestamp_ns, sensor_name, data.angular_velocity, data.linear_acceleration)};

    return Sqlite3Tools::Execute(insert_imu_data_sql, database->db);
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
        std::cerr << "Failed to encode image as PNG" << "\n";  // LCOV_EXCL_LINE
        return false;                                          // LCOV_EXCL_LINE
    }

    std::string const insert_image_sql{InsertImageSql(sensor_name, data.timestamp_ns)};

    return Sqlite3Tools::AddBlob(insert_image_sql, buffer.data(), std::size(buffer), database->db);
}

ImageStreamer::ImageStreamer(std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name,
                             uint64_t const start_time)
    : database_{database} {
    std::string const image_streamer_sql{ImageStreamerSql(sensor_name, start_time)};

    int code{sqlite3_prepare_v2(database_->db, image_streamer_sql.c_str(), -1, &stmt_, nullptr)};
    if (code != static_cast<int>(SqliteFlag::Ok)) {
        throw std::runtime_error("Add image sqlite3_prepare_v2() failed: " +   // LCOV_EXCL_LINE
                                 std::string{sqlite3_errmsg(database_->db)});  // LCOV_EXCL_LINE
    }
}

ImageStreamer::~ImageStreamer() { sqlite3_finalize(stmt_); }

std::optional<ImageData> ImageStreamer::Next() {
    int const code{sqlite3_step(stmt_)};
    if (code == static_cast<int>(SqliteFlag::Done)) {
        return std::nullopt;
    } else if (code != static_cast<int>(SqliteFlag::Row)) {
        std::cerr << "Error stepping: " << sqlite3_errmsg(database_->db) << "\n";  // LCOV_EXCL_LINE
        return std::nullopt;                                                       // LCOV_EXCL_LINE
    }

    // TODO(Jack): Should we be more defensive here and first check that column text is not returning a nullptr or other
    // bad output?
    uint64_t const timestamp_ns{std::stoull(reinterpret_cast<const char*>(sqlite3_column_text(stmt_, 0)))};

    uchar const* const blob{static_cast<uchar const*>(sqlite3_column_blob(stmt_, 1))};
    int const blob_size{sqlite3_column_bytes(stmt_, 1)};
    if (not blob or blob_size <= 0) {
        std::cerr << "Empty blob for timestamp: " << timestamp_ns << "\n";  // LCOV_EXCL_LINE
        return std::nullopt;                                                // LCOV_EXCL_LINE
    }

    std::vector<uchar> const buffer(blob, blob + blob_size);
    cv::Mat const image{cv::imdecode(buffer, cv::IMREAD_UNCHANGED)};
    if (image.empty()) {
        std::cerr << "Failed to decode PNG for timestamp: " << timestamp_ns << "\n";  // LCOV_EXCL_LINE
        return std::nullopt;                                                          // LCOV_EXCL_LINE
    }

    return ImageData{timestamp_ns, image};
}

};  // namespace reprojection::database