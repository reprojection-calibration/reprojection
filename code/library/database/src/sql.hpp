#pragma once

#include <cstdint>
#include <string>

namespace reprojection::database {

inline std::string ImageStreamerSql(std::string const& sensor_name, uint64_t const start_time) {
    std::string const sql{
        "SELECT timestamp_ns, data "
        "FROM images "
        "WHERE sensor_name = '" +
        sensor_name + "' AND timestamp_ns >= " + std::to_string(start_time) + " ORDER BY timestamp_ns ASC;"};

    return sql;
}

}  // namespace reprojection::database