#pragma once

#include <cstdint>
#include <string>

namespace reprojection::database {

// TODO REMOVE AFTER REFACTOR
inline std::string InsertExtractedTargetSql(std::string const& sensor_name, uint64_t const timestamp_ns) {
    std::string const sql{
        "INSERT INTO extracted_targets (timestamp_ns, sensor_name, data) "
        "VALUES(" +
        std::to_string(timestamp_ns) + ", '" + sensor_name + "', ?)"};

    return sql;
}

// NOTE(Jack): Assumes that we can load all target into memory not problem - no need for streaming like we need to do
// for the images.
inline std::string SelectExtractedTargetDataSql(std::string const& sensor_name) {
    std::string const sql{
        "SELECT timestamp_ns, data "
        "FROM extracted_targets "
        "WHERE sensor_name = '" +
        sensor_name + "' ORDER BY timestamp_ns ASC;"};

    return sql;
}

inline std::string InsertImuDataSql(std::uint64_t const timestamp_ns, std::string const& sensor_name,
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

// ERROR(Jack): Risk of sql injection here?
inline std::string SelectImuSensorDataSql(std::string const& sensor_name) {
    std::string const sql{
        "SELECT timestamp_ns, omega_x, omega_y, omega_z, ax, ay, az "
        "FROM imu_data "
        "WHERE sensor_name = '" +
        sensor_name +
        "' "
        "ORDER BY timestamp_ns ASC;"};

    return sql;
}

inline const std::string images_table_sql{
    "CREATE TABLE IF NOT EXISTS images ("
    "timestamp_ns INTEGER NOT NULL, "
    "sensor_name TEXT NOT NULL, "
    "data BLOB NOT NULL, "
    "PRIMARY KEY (timestamp_ns, sensor_name));"};

inline std::string InsertImageSql(std::string const& sensor_name, uint64_t const timestamp_ns) {
    std::string const sql{
        "INSERT INTO images (timestamp_ns, sensor_name, data) "
        "VALUES(" +
        std::to_string(timestamp_ns) + ", '" + sensor_name + "', ?)"};

    return sql;
}

inline std::string ImageStreamerSql(std::string const& sensor_name, uint64_t const start_time) {
    std::string const sql{
        "SELECT timestamp_ns, data "
        "FROM images "
        "WHERE sensor_name = '" +
        sensor_name + "' AND timestamp_ns >= " + std::to_string(start_time) + " ORDER BY timestamp_ns ASC;"};

    return sql;
}

}  // namespace reprojection::database