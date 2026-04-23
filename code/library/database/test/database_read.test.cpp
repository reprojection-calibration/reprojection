#include "database/database_read.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "testing_utilities/constants.hpp"
#include "types/sensor_data_types.hpp"

using namespace reprojection;

class CameraReadFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = database::OpenCalibrationDatabase(":memory:", true, false);

        database::WriteToDb(CalibrationStep::CameraInfo, "", sensor_name, db);
        database::WriteToDb(CameraInfo{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds}, db);
    }

    void AddImage(uint64_t const timestamp_ns) const {
        // Due to foreign key relationship we need add an image before we add the target
        database::WriteToDb(CalibrationStep::ImageLoading, "", sensor_name, db);
        database::WriteToDb(EncodedImages{{timestamp_ns, {}}}, sensor_name, db);
    }

    void AddTarget(uint64_t const timestamp_ns) const {
        // Due to foreign key relationship we need add an image before we add the target
        AddImage(timestamp_ns);

        database::WriteToDb(CalibrationStep::FeatureExtraction, "", sensor_name, db);
        database::WriteToDb({{timestamp_ns, target}}, sensor_name, db);
    }

    SqlitePtr db{nullptr};
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

TEST_F(CameraReadFixture, TestGetEncodedImages) {
    AddImage(0);
    AddImage(1);
    AddImage(2);

    EncodedImages const loaded_data{database::GetEncodedImages(db, sensor_name)};
    EXPECT_EQ(std::size(loaded_data), 3);

    int test_timestamp{0};
    for (auto const& [timestamp_ns_i, buffer] : loaded_data) {
        EXPECT_EQ(timestamp_ns_i, test_timestamp);
        test_timestamp += 1;

        // TODO(Jack): We should consider making the absent of an image explicit with std::optional.
        EXPECT_EQ(std::size(buffer.data), 0);
    }
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

TEST_F(CameraReadFixture, TestReadCameraState) {
    auto const step{CalibrationStep::Lpi};

    auto intrinsics{database::ReadCameraState(db, step, "", CameraModel::Pinhole)};
    EXPECT_FALSE(intrinsics.has_value());

    database::WriteToDb(CalibrationStep::Lpi, "", sensor_name, db);
    database::WriteToDb({testing_utilities::pinhole_intrinsics}, CameraModel::Pinhole, step, sensor_name, db);

    intrinsics = database::ReadCameraState(db, step, sensor_name, CameraModel::Pinhole);
    ASSERT_TRUE(intrinsics.has_value());
    EXPECT_TRUE(intrinsics->isApprox(testing_utilities::pinhole_intrinsics));
}

TEST_F(CameraReadFixture, TestReadCacheKey) {
    auto cache_key{database::ReadCacheKey(db, CalibrationStep::Lpi, sensor_name)};
    EXPECT_FALSE(cache_key.has_value());

    database::WriteToDb(CalibrationStep::Lpi, "1", sensor_name, db);

    cache_key = database::ReadCacheKey(db, CalibrationStep::Lpi, sensor_name);
    ASSERT_TRUE(cache_key.has_value());
    EXPECT_EQ(cache_key.value(), "1");

    database::WriteToDb(CalibrationStep::Lpi, "2", sensor_name, db);

    cache_key = database::ReadCacheKey(db, CalibrationStep::Lpi, sensor_name);
    ASSERT_TRUE(cache_key.has_value());
    EXPECT_EQ(cache_key.value(), "2");
}

TEST_F(CameraReadFixture, TestReadPoses) {
    auto const step{CalibrationStep::Lpi};
    uint64_t const timestamp_ns{0};

    Frames result{database::ReadPoses(db, step, sensor_name)};
    EXPECT_EQ(std::size(result), 0);

    database::WriteToDb(step, "", sensor_name, db);
    Frames const frames{{timestamp_ns, {Array6d::Zero()}}, {timestamp_ns + 1, {Array6d::Zero()}}};
    database::WriteToDb(frames, step, sensor_name, db);

    result = database::ReadPoses(db, step, sensor_name);
    EXPECT_EQ(std::size(result), 2);
    EXPECT_TRUE(result.at(timestamp_ns).pose.isApprox(frames.at(timestamp_ns).pose));
}

TEST(DatabaseSensorDataInterface, TestFullImuAddGetCycle) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    std::string_view sensor_name{"/imu/polaris/123"};
    ImuMeasurements const data{{0, {Vector3d::Zero(), Vector3d::Zero()}},  //
                               {1, {Vector3d::Zero(), Vector3d::Zero()}},
                               {2, {Vector3d::Zero(), Vector3d::Zero()}}};

    EXPECT_NO_THROW(database::WriteToDb(data, sensor_name, db));

    auto const loaded_data{database::GetImuData(db, sensor_name)};
    EXPECT_EQ(std::size(loaded_data), std::size(data));
}

TEST(DatabaseSensorDataInterface, TestGetImuData) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    // Data from imu 123
    std::string_view sensor_name_1{"/imu/polaris/123"};
    database::WriteToDb({{5, {{1, 2, 3}, {4, 5, 6}}},  //
                         {10, {Vector3d::Zero(), Vector3d::Zero()}},
                         {15, {Vector3d::Zero(), Vector3d::Zero()}}},
                        sensor_name_1, db);
    // Data from imu 456
    std::string_view sensor_name_2{"/imu/polaris/456"};
    database::WriteToDb(ImuMeasurements{{10, {Vector3d::Zero(), Vector3d::Zero()}},  //
                                        {20, {Vector3d::Zero(), Vector3d::Zero()}}},
                        sensor_name_2, db);

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
