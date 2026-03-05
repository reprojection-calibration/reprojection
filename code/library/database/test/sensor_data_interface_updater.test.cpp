#include "database/sensor_data_interface_updater.hpp"

#include <gtest/gtest.h>

#include <string>

#include "database/calibration_database.hpp"
#include "database/sensor_data_interface_adders.hpp"
#include "database/sensor_data_interface_getters.hpp"

using namespace reprojection;

TEST(DatabaseSensorDataInterface, TestUpdateCalibrationStep) {
    auto db{std::make_shared<database::CalibrationDatabase>(":memory:", true, false)};

    std::string const step_name{"linear_pose_initialization"};

    database::AddCalibrationStep(step_name, "key0", db);
    database::UpdateCalibrationStep(step_name, "key1", db);

    auto const key{database::GetCacheKey(db, step_name)};
    ASSERT_TRUE(key.has_value());
    EXPECT_EQ(key.value(), "key1");
}
