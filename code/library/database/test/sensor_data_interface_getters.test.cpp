#include "database/sensor_data_interface_getters.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "database/image_interface.hpp"
#include "testing_utilities/constants.hpp"
#include "types/sensor_data_types.hpp"

using namespace reprojection;

class CameraReadFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = std::make_shared<database::CalibrationDatabase>(":memory:", true, false);

        database::WriteToDb(CameraInfo{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds}, db);
    }

    void AddTarget(uint64_t const timestamp_ns) const {
        // Due to foreign key relationship we need add an image before we add the target
        database::AddImage(timestamp_ns, sensor_name, db);
        database::WriteToDb(sensor_name, {timestamp_ns, target}, db);
    }

    std::shared_ptr<database::CalibrationDatabase> db;
    std::string sensor_name{"/cam/retro/123"};
    ExtractedTarget target{Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}, {200.24, 300.56}},
                                  MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}, {300.65, 200.56, 712.57}}},
                           {{5, 6}, {2, 3}, {650, 600}}};
};

TEST_F(CameraReadFixture, TestReadCameraInfo) {
    auto camera_info{database::ReadCameraInfo(db, "/nonexistent/camera")};
    EXPECT_FALSE(camera_info.has_value());

    camera_info = database::ReadCameraInfo(db, sensor_name);
    ASSERT_TRUE(camera_info.has_value());
    EXPECT_EQ(camera_info->sensor_name, sensor_name);
    EXPECT_EQ(camera_info->camera_model, CameraModel::Pinhole);
    EXPECT_EQ(camera_info->bounds.u_min, testing_utilities::image_bounds.u_min);
    EXPECT_EQ(camera_info->bounds.u_max, testing_utilities::image_bounds.u_max);
    EXPECT_EQ(camera_info->bounds.v_min, testing_utilities::image_bounds.v_min);
    EXPECT_EQ(camera_info->bounds.v_max, testing_utilities::image_bounds.v_max);
}

TEST_F(CameraReadFixture, TestGetExtractedTargetData) {
    AddTarget(0);
    AddTarget(1);
    AddTarget(2);

    CameraMeasurements const loaded_data{database::GetExtractedTargetData(db, sensor_name)};
    EXPECT_EQ(std::size(loaded_data), 3);

    int test_timestamp{0};
    for (auto const& [timestamp_ns_i, target_i] : loaded_data) {
        EXPECT_EQ(timestamp_ns_i, test_timestamp);
        test_timestamp += 1;

        EXPECT_TRUE(target_i.bundle.pixels.isApprox(target.bundle.pixels));
        EXPECT_TRUE(target_i.bundle.points.isApprox(target.bundle.points));
        EXPECT_TRUE(target_i.indices.isApprox(target.indices));
    }
}

TEST_F(CameraReadFixture, TestReadCacheKey) {
    auto cache_key{database::ReadCacheKey(db, CalibrationStep::Lpi, sensor_name)};
    EXPECT_FALSE(cache_key.has_value());

    WriteToDb(CalibrationStep::Lpi, sensor_name, "1", db);

    cache_key = database::ReadCacheKey(db, CalibrationStep::Lpi, sensor_name);
    ASSERT_TRUE(cache_key.has_value());
    EXPECT_EQ(cache_key.value(), "1");

    WriteToDb(CalibrationStep::Lpi, sensor_name, "2", db);

    cache_key = database::ReadCacheKey(db, CalibrationStep::Lpi, sensor_name);
    ASSERT_TRUE(cache_key.has_value());
    EXPECT_EQ(cache_key.value(), "2");
}

TEST(DatabaseSensorDataInterface, TestFullImuAddGetCycle) {
    auto const db{std::make_shared<database::CalibrationDatabase>(":memory:", true, false)};

    std::string_view sensor_name{"/imu/polaris/123"};
    ImuMeasurements const data{{0, {Vector3d::Zero(), Vector3d::Zero()}},  //
                               {1, {Vector3d::Zero(), Vector3d::Zero()}},
                               {2, {Vector3d::Zero(), Vector3d::Zero()}}};

    EXPECT_NO_THROW(database::WriteToDb(sensor_name, data, db));

    auto const loaded_data{database::GetImuData(db, sensor_name)};
    EXPECT_EQ(std::size(loaded_data), std::size(data));
}

TEST(DatabaseSensorDataInterface, TestGetImuData) {
    auto const db{std::make_shared<database::CalibrationDatabase>(":memory:", true)};

    // Data from imu 123
    std::string_view sensor_name_1{"/imu/polaris/123"};
    database::WriteToDb(sensor_name_1,
                        {{5, {{1, 2, 3}, {4, 5, 6}}},  //
                         {10, {Vector3d::Zero(), Vector3d::Zero()}},
                         {15, {Vector3d::Zero(), Vector3d::Zero()}}},
                        db);
    // Data from imu 456
    std::string_view sensor_name_2{"/imu/polaris/456"};
    database::WriteToDb(sensor_name_2,
                        {{10, {Vector3d::Zero(), Vector3d::Zero()}},  //
                         {20, {Vector3d::Zero(), Vector3d::Zero()}}},
                        db);

    auto const imu_1_data{database::GetImuData(db, sensor_name_1)};
    EXPECT_EQ(std::size(imu_1_data), 3);

    // Check the values of the first element to make sure the callback lambda reading logic is correct
    ImuData const imu_data_i{imu_1_data.at(5)};
    EXPECT_EQ(imu_data_i.angular_velocity[0], 1);
    EXPECT_EQ(imu_data_i.angular_velocity[1], 2);
    EXPECT_EQ(imu_data_i.angular_velocity[2], 3);
    EXPECT_EQ(imu_data_i.linear_acceleration[0], 4);
    EXPECT_EQ(imu_data_i.linear_acceleration[1], 5);
    EXPECT_EQ(imu_data_i.linear_acceleration[2], 6);

    auto const imu_2_data{database::GetImuData(db, sensor_name_2)};
    EXPECT_EQ(std::size(imu_2_data), 2);

    // If the sensor is not present we simply get an empty set back, this is not an error
    auto const unknown_sensor_data{database::GetImuData(db, "/imu/polaris/unknown")};
    EXPECT_EQ(std::size(unknown_sensor_data), 0);
}
