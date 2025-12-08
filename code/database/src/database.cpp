
#include "database/database.hpp"

#include <sqlite3.h>

#include <filesystem>
#include <string>

#include "sqlite3_helpers.hpp"

namespace reprojection::database {

CalibrationDatabase::CalibrationDatabase(std::string const& db_path, bool const create, bool const read_only) {
    if (create and read_only) {
        throw std::runtime_error(
            "You requested to open a database object with both options 'create' and 'read_only' true. This is "
            "an invalid combination as creating a database requires writing to it!");
    }

    // TODO(Jack): Consider using sqlite3_errcode for better terminal output https://sqlite.org/c3ref/errcode.html
    int code;
    if (create) {
        // WARN(Jack): Should it be an error if create is true and the database already exists. Is that a problem?
        code = sqlite3_open_v2(db_path.c_str(), &db_,
                               static_cast<int>(SqliteFlag::OpenReadWrite) | static_cast<int>(SqliteFlag::OpenCreate),
                               nullptr);
    } else if (read_only) {
        code = sqlite3_open_v2(db_path.c_str(), &db_, static_cast<int>(SqliteFlag::OpenReadOnly), nullptr);
    } else {
        code = sqlite3_open_v2(db_path.c_str(), &db_, static_cast<int>(SqliteFlag::OpenReadWrite), nullptr);
    }

    if (code != 0) {
        sqlite3_close(db_);
        throw std::runtime_error("Attempted to open database at path - " + db_path + " - but was unsuccessful");
    }
}

CalibrationDatabase::~CalibrationDatabase() { sqlite3_close(db_); }

[[nodiscard]] bool CalibrationDatabase::AddImuData(std::string const& sensor_name, ImuData const& data) {
    // TODO(Jack): This is hacky! We need to find a principle way to create all data tables, not recreate it
    // everytime we want to add data!
    // TODO(Jack): Is there any problem we will have storing our time type (uint64t) here as a signed integer type?
    // TODO(Jack): Should index the sensor by an integer id or by a string?
    std::string const init_imu_table{
        "CREATE TABLE IF NOT EXISTS imu_data ("
        "timestamp_ns INTEGER NOT NULL, "
        "sensor_name TEXT NOT NULL, "
        "omega_x REAL NOT NULL, "
        "omega_y REAL NOT NULL, "
        "omega_z REAL NOT NULL, "
        "ax REAL NOT NULL, "
        "ay REAL NOT NULL, "
        "az REAL NOT NULL, "
        "PRIMARY KEY (timestamp_ns, sensor_name)"
        ");"};
    bool success{Sqlite3Tools::Execute(init_imu_table, db_)};
    if (not success) {
        return false;
    }

    std::string const add_imu_row{
        "INSERT INTO imu_data (timestamp_ns, sensor_name, omega_x, omega_y, omega_z, ax, ay, az) "
        "VALUES (" +
        std::to_string(data.timestamp_ns) + ", '" + sensor_name + "', " + std::to_string(data.angular_velocity[0]) + ", " +
        std::to_string(data.angular_velocity[1]) + ", " + std::to_string(data.angular_velocity[2]) + ", " +
        std::to_string(data.linear_acceleration[0]) + ", " + std::to_string(data.linear_acceleration[1]) + ", " +
        std::to_string(data.linear_acceleration[2]) + ");"};

    return Sqlite3Tools::Execute(add_imu_row, db_);
}

};  // namespace reprojection::database