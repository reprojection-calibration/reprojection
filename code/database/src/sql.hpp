#pragma once

#include <cstdint>
#include <string>

namespace reprojection::database {

// TODO(Jack): Is there any problem we will have storing our time type (uint64t) here as a signed integer type?
// TODO(Jack): Should index the sensor by an integer id or by a string?
inline const std::string imu_table_sql{
    "CREATE TABLE IF NOT EXISTS imu_data ("
    "timestamp_ns INTEGER NOT NULL, "
    "sensor_name TEXT NOT NULL, "
    "omega_x REAL NOT NULL, "
    "omega_y REAL NOT NULL, "
    "omega_z REAL NOT NULL, "
    "ax REAL NOT NULL, "
    "ay REAL NOT NULL, "
    "az REAL NOT NULL, "
    "PRIMARY KEY (timestamp_ns, sensor_name));"};

std::string InsertImuDataSql(std::uint64_t const timestamp_ns, std::string const& sensor_name,
                             double const angular_velocity[3], double const linear_acceleration[3]) {
    std::string const sql{
        "INSERT INTO imu_data (timestamp_ns, sensor_name, omega_x, omega_y, omega_z, ax, ay, az) "
        "VALUES (" +
        std::to_string(timestamp_ns) + ", '" + sensor_name + "', " + std::to_string(angular_velocity[0]) + ", " +
        std::to_string(angular_velocity[1]) + ", " + std::to_string(angular_velocity[2]) + ", " +
        std::to_string(linear_acceleration[0]) + ", " + std::to_string(linear_acceleration[1]) + ", " +
        std::to_string(linear_acceleration[2]) + ");"};

    return sql;
}
std::string SelectImuSensorDataSql(std::string const& sensor_name) {
    std::string const sql{
        "SELECT timestamp_ns, omega_x, omega_y, omega_z, ax, ay, az "
        "FROM imu_data "
        "WHERE sensor_name = '" +
        sensor_name +
        "' "
        "ORDER BY timestamp_ns ASC;"};

    return sql;
}

}  // namespace reprojection::database