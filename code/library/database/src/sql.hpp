#pragma once

#include <cstdint>
#include <string>

namespace reprojection::database {

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

inline std::string ImageStreamerSql(std::string const& sensor_name, uint64_t const start_time) {
    std::string const sql{
        "SELECT timestamp_ns, data "
        "FROM images "
        "WHERE sensor_name = '" +
        sensor_name + "' AND timestamp_ns >= " + std::to_string(start_time) + " ORDER BY timestamp_ns ASC;"};

    return sql;
}

}  // namespace reprojection::database