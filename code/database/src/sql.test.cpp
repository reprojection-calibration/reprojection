#include "sql.hpp"

#include <gtest/gtest.h>

#include <string>

using namespace reprojection;

TEST(DatabaseSql, TestInsertImuDataSql) {
    double const angular_velocity[3]{1, 2, 3};
    double const linear_acceleration[3]{4, 5, 6};
    std::string const sql{
        database::InsertImuDataSql(1765265480996386555, "abcdefg", angular_velocity, linear_acceleration)};

    EXPECT_EQ(sql,
              "INSERT INTO imu_data (timestamp_ns, sensor_name, omega_x, omega_y, omega_z, ax, ay, az) VALUES "
              "(1765265480996386555, 'abcdefg', 1.000000, 2.000000, 3.000000, 4.000000, 5.000000, 6.000000);");
}

TEST(DatabaseSql, TestSelectImuSensorDataSql) {
    std::string const sql{database::SelectImuSensorDataSql("abcdefg")};

    EXPECT_EQ(sql,
              "SELECT timestamp_ns, omega_x, omega_y, omega_z, ax, ay, az FROM imu_data WHERE sensor_name = 'abcdefg' "
              "ORDER BY timestamp_ns ASC;");
}