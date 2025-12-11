#include "sql.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace reprojection;

TEST(DatabaseSql, TestSelectImuSensorDataSql) {
    std::string const sql{database::SelectImuSensorDataSql("abcdefg")};

    EXPECT_EQ(sql,
              "SELECT timestamp_ns, omega_x, omega_y, omega_z, ax, ay, az FROM imu_data WHERE sensor_name = 'abcdefg' "
              "ORDER BY timestamp_ns ASC;");
}