#include "database/sensor_data_interface.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <memory>
#include <string>

#include "database/sqlite_wrappers.hpp"
// cppcheck-suppress missingInclude
#include "generated/sql.hpp"
#include "serialization.hpp"
#include "sqlite3_helpers.hpp"

namespace reprojection::database {

[[nodiscard]] bool AddPoseData(std::set<PoseStamped> const& data, PoseTable const table, PoseType const type,
                               std::shared_ptr<CalibrationDatabase> const database) {
    SqlTransaction const lock{(database->db)};

    for (auto const& data_i : data) {
        std::unique_ptr<SqlStatement> statement;
        if (table == PoseTable::Camera) {
            statement = std::make_unique<SqlStatement>(database->db, sql_statements::camera_poses_insert);
        } else if (table == PoseTable::External) {
            statement = std::make_unique<SqlStatement>(database->db, sql_statements::external_poses_insert);
        } else {
            throw std::runtime_error("Requested an invalid PoseTable from AddPoseData()");  // LCOV_EXCL_LINE
        }

        try {
            Sqlite3Tools::Bind(statement->stmt, 1, static_cast<int64_t>(data_i.header.timestamp_ns));  // Warn cast!
            Sqlite3Tools::Bind(statement->stmt, 2, data_i.header.sensor_name);
            Sqlite3Tools::Bind(statement->stmt, 3, ToString(type));
            Sqlite3Tools::Bind(statement->stmt, 4, data_i.pose[0]);
            Sqlite3Tools::Bind(statement->stmt, 5, data_i.pose[1]);
            Sqlite3Tools::Bind(statement->stmt, 6, data_i.pose[2]);
            Sqlite3Tools::Bind(statement->stmt, 7, data_i.pose[3]);
            Sqlite3Tools::Bind(statement->stmt, 8, data_i.pose[4]);
            Sqlite3Tools::Bind(statement->stmt, 9, data_i.pose[5]);
        } catch (std::runtime_error const& e) {                                                     // LCOV_EXCL_LINE
            std::cerr << "AddPoseData() runtime error during binding: " << e.what()                 // LCOV_EXCL_LINE
                      << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
            return false;                                                                           // LCOV_EXCL_LINE
        }  // LCOV_EXCL_LINE

        if (sqlite3_step(statement->stmt) != static_cast<int>(SqliteFlag::Done)) {
            std::cerr << "AddPoseData() sqlite3_step() failed: " << sqlite3_errmsg(database->db) << "\n";
            return false;
        }
    }

    return true;
}

[[nodiscard]] bool AddExtractedTargetData(ExtractedTargetStamped const& data,
                                          std::shared_ptr<CalibrationDatabase> const database) {
    protobuf_serialization::ExtractedTargetProto const serialized{Serialize(data.target)};
    std::string buffer;
    if (not serialized.SerializeToString(&buffer)) {
        std::cerr << "ExtractedTarget serialization failed at: "
                  << std::to_string(data.header.timestamp_ns)  // LCOV_EXCL_LINE
                  << "\n";                                     // LCOV_EXCL_LINE
        return false;                                          // LCOV_EXCL_LINE
    }

    return Sqlite3Tools::AddBlob(sql_statements::extracted_target_insert, data.header.timestamp_ns,
                                 data.header.sensor_name, buffer.c_str(), std::size(buffer), database->db);
}

// NOTE(Jack): The logic here is very similar to the ImageStreamer class, but there are enough differences that we
// cannot easily reconcile the two and eliminate copy and past like we did for the Add* functions.
std::optional<std::set<ExtractedTargetStamped>> GetExtractedTargetData(
    std::shared_ptr<CalibrationDatabase const> const database, std::string const& sensor_name) {
    SqlStatement const statement{database->db, sql_statements::extracted_targets_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, sensor_name.c_str());
    } catch (std::runtime_error const& e) {                                                     // LCOV_EXCL_LINE
        std::cerr << "GetExtractedTargetData() runtime error during binding: " << e.what()      // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        return std::nullopt;                                                                    // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    std::set<ExtractedTargetStamped> data;
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

        data.insert(ExtractedTargetStamped{{timestamp_ns, sensor_name}, deserialized.value()});
    }

    return data;
}

[[nodiscard]] bool AddImuData(ImuStamped const& data, std::shared_ptr<CalibrationDatabase> const database) {
    SqlStatement const statement{database->db, sql_statements::imu_data_insert};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, static_cast<int64_t>(data.header.timestamp_ns));  // Warn cast!
        Sqlite3Tools::Bind(statement.stmt, 2, data.header.sensor_name);
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

std::optional<std::set<ImuStamped>> GetImuData(std::shared_ptr<CalibrationDatabase const> const database,
                                               std::string const& sensor_name) {
    SqlStatement const statement{database->db, sql_statements::imu_data_select};

    try {
        Sqlite3Tools::Bind(statement.stmt, 1, sensor_name.c_str());
    } catch (std::runtime_error const& e) {                                                     // LCOV_EXCL_LINE
        std::cerr << "GetImuData() runtime error during binding: " << e.what()                  // LCOV_EXCL_LINE
                  << " with database error message: " << sqlite3_errmsg(database->db) << "\n";  // LCOV_EXCL_LINE
        return std::nullopt;                                                                    // LCOV_EXCL_LINE
    }  // LCOV_EXCL_LINE

    std::set<ImuStamped> data;
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

        data.insert(ImuStamped{{timestamp_ns, sensor_name}, {omega_x, omega_y, omega_z}, {ax, ay, az}});
    }

    return data;
}

};  // namespace reprojection::database