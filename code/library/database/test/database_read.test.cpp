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

        database::InsertStep(CalibrationStep::CameraInfo, "", sensor_name, db);
        database::InsertCameraInfo(CameraInfo{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds}, db);
    }

    void AddImage(uint64_t const timestamp_ns) const {
        // Due to foreign key relationship we need add an image before we add the target
        database::InsertStep(CalibrationStep::ImageLoading, "", sensor_name, db);
        database::InsertImages(EncodedImages{{timestamp_ns, {}}}, sensor_name, db);
    }

    void AddTarget(uint64_t const timestamp_ns) const {
        // Due to foreign key relationship we need add an image before we add the target
        AddImage(timestamp_ns);

        database::InsertStep(CalibrationStep::FeatureExtraction, "", sensor_name, db);
        database::InsertTargets({{timestamp_ns, target}}, sensor_name, db);
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

TEST_F(CameraReadFixture, TestReadTargetInfo) {
    auto target_info{database::ReadTargetInfo(db, "/nonexistent/camera")};
    EXPECT_FALSE(target_info.has_value());

    database::InsertStep(CalibrationStep::TargetInfo, "", sensor_name, db);
    database::InsertTargetInfo(TargetInfo{TargetType::Aprilgrid3, 8, 6, 0.1, false}, sensor_name, db);

    target_info = database::ReadTargetInfo(db, sensor_name);
    ASSERT_TRUE(target_info.has_value());
    EXPECT_EQ(target_info->target_type, TargetType::Aprilgrid3);
    EXPECT_EQ(target_info->height, 8);
    EXPECT_EQ(target_info->width, 6);
    EXPECT_EQ(target_info->unit_dimension, 0.1);
    EXPECT_EQ(target_info->asymmetric, false);
}

TEST_F(CameraReadFixture, TestGetEncodedImages) {
    AddImage(0);
    AddImage(1);
    AddImage(2);

    EncodedImages const loaded_data{database::ReadEncodedImages(db, sensor_name)};
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

    CameraMeasurements const loaded_data{database::ReadExtractedTargets(db, sensor_name)};
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
    auto const step{CalibrationStep::PoseInitialization};

    auto intrinsics{database::ReadCameraState(db, step, "", CameraModel::Pinhole)};
    EXPECT_FALSE(intrinsics.has_value());

    database::InsertStep(CalibrationStep::PoseInitialization, "", sensor_name, db);
    database::InsertIntrinsics({testing_utilities::pinhole_intrinsics}, CameraModel::Pinhole, step, sensor_name, db);

    intrinsics = database::ReadCameraState(db, step, sensor_name, CameraModel::Pinhole);
    ASSERT_TRUE(intrinsics.has_value());
    EXPECT_TRUE(intrinsics->isApprox(testing_utilities::pinhole_intrinsics));
}

TEST_F(CameraReadFixture, TestReadCacheKey) {
    auto cache_key{database::ReadCacheKey(db, CalibrationStep::PoseInitialization, sensor_name)};
    EXPECT_FALSE(cache_key.has_value());

    database::InsertStep(CalibrationStep::PoseInitialization, "1", sensor_name, db);

    cache_key = database::ReadCacheKey(db, CalibrationStep::PoseInitialization, sensor_name);
    ASSERT_TRUE(cache_key.has_value());
    EXPECT_EQ(cache_key.value(), "1");

    database::InsertStep(CalibrationStep::PoseInitialization, "2", sensor_name, db);

    cache_key = database::ReadCacheKey(db, CalibrationStep::PoseInitialization, sensor_name);
    ASSERT_TRUE(cache_key.has_value());
    EXPECT_EQ(cache_key.value(), "2");
}

TEST_F(CameraReadFixture, TestReadPoses) {
    auto const step{CalibrationStep::PoseInitialization};
    uint64_t const timestamp_ns{0};

    // No matching frames in the database means we get an empty container.
    Frames result{database::ReadPoses(db, step, sensor_name)};
    EXPECT_EQ(std::size(result), 0);

    // Satisfy the foreign key constraints for adding a camera frame pose.
    AddTarget(timestamp_ns);
    database::InsertStep(step, "", sensor_name, db);

    // Add one frame to the database then load it and see that it's the same.
    Frames const frames{{timestamp_ns, {Array6d::Zero()}}};
    database::InsertPoses(frames, step, sensor_name, db);

    result = database::ReadPoses(db, step, sensor_name);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_TRUE(result.at(timestamp_ns).pose.isApprox(frames.at(timestamp_ns).pose));
}

TEST(DatabaseDatabaseRead, TestFullImuAddGetCycle) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true, false)};

    std::string_view sensor_name{"/imu/polaris/123"};
    ImuMeasurements const data{{0, {Vector3d::Zero(), Vector3d::Zero()}},  //
                               {1, {Vector3d::Zero(), Vector3d::Zero()}},
                               {2, {Vector3d::Zero(), Vector3d::Zero()}}};

    EXPECT_NO_THROW(database::InsertImuData(data, sensor_name, db));

    auto const loaded_data{database::ReadImuData(db, sensor_name)};
    EXPECT_EQ(std::size(loaded_data), std::size(data));
}

TEST(DatabaseDatabaseRead, TestGetImuData) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    // Data from imu 123
    std::string_view sensor_name_1{"/imu/polaris/123"};
    database::InsertImuData({{5, {{1, 2, 3}, {4, 5, 6}}},  //
                         {10, {Vector3d::Zero(), Vector3d::Zero()}},
                         {15, {Vector3d::Zero(), Vector3d::Zero()}}},
                        sensor_name_1, db);
    // Data from imu 456
    std::string_view sensor_name_2{"/imu/polaris/456"};
    database::InsertImuData(ImuMeasurements{{10, {Vector3d::Zero(), Vector3d::Zero()}},  //
                                        {20, {Vector3d::Zero(), Vector3d::Zero()}}},
                        sensor_name_2, db);

    auto const imu_1_data{database::ReadImuData(db, sensor_name_1)};
    EXPECT_EQ(std::size(imu_1_data), 3);

    // Check the values of the first element to make sure the callback lambda reading logic is correct
    ImuData const imu_data_i{imu_1_data.at(5)};
    EXPECT_EQ(imu_data_i.angular_velocity[0], 1);
    EXPECT_EQ(imu_data_i.angular_velocity[1], 2);
    EXPECT_EQ(imu_data_i.angular_velocity[2], 3);
    EXPECT_EQ(imu_data_i.linear_acceleration[0], 4);
    EXPECT_EQ(imu_data_i.linear_acceleration[1], 5);
    EXPECT_EQ(imu_data_i.linear_acceleration[2], 6);

    auto const imu_2_data{database::ReadImuData(db, sensor_name_2)};
    EXPECT_EQ(std::size(imu_2_data), 2);

    // If the sensor is not present we simply get an empty set back, this is not an error
    auto const unknown_sensor_data{database::ReadImuData(db, "/imu/polaris/unknown")};
    EXPECT_EQ(std::size(unknown_sensor_data), 0);
}

TEST(DatabaseDatabaseRead, TestReadImuErrors) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};
    std::string_view sensor_name{"/imu/polaris/123"};

    // Satisfy foreign key constraints and write the imu errors to the database so we can load them.
    database::InsertImuData({{5, {{1, 2, 3}, {4, 5, 6}}},  //
                         {10, {Vector3d::Zero(), Vector3d::Zero()}},
                         {15, {Vector3d::Zero(), Vector3d::Zero()}}},
                        sensor_name, db);
    database::InsertStep(CalibrationStep::ExtrinsicInitialization, "", sensor_name, db);

    database::InsertImuErrors(ImuErrors{{5, {{1, 2, 3}, {4, 5, 6}}},  //
                                  {10, {Vector3d::Zero(), Vector3d::Zero()}},
                                  {15, {Vector3d::Zero(), Vector3d::Zero()}}},
                        CalibrationStep::ExtrinsicInitialization, sensor_name, db);

    // Load the errors and check their size and the values in the first one.
    auto const imu_errors{database::ReadImuErrors(db, CalibrationStep::ExtrinsicInitialization, sensor_name)};
    EXPECT_EQ(std::size(imu_errors), 3);

    ImuErrorState const imu_error_i{imu_errors.at(5)};
    EXPECT_EQ(imu_error_i.delta_angular_velocity[0], 1);
    EXPECT_EQ(imu_error_i.delta_angular_velocity[1], 2);
    EXPECT_EQ(imu_error_i.delta_angular_velocity[2], 3);
    EXPECT_EQ(imu_error_i.delta_linear_acceleration[0], 4);
    EXPECT_EQ(imu_error_i.delta_linear_acceleration[1], 5);
    EXPECT_EQ(imu_error_i.delta_linear_acceleration[2], 6);

    // Try to read data that does not exist - this simply returns an empty container and is NOT an error.
    auto const unknown_sensor_data{database::ReadImuData(db, "/imu/polaris/unknown")};
    EXPECT_EQ(std::size(unknown_sensor_data), 0);
}

TEST(DatabaseDatabaseRead, TestReadSplineControlPoints) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    std::string_view sensor_name{"/cam/retro/123"};
    database::InsertStep(CalibrationStep::SplineInterpolation, "", sensor_name, db);
    spline::Matrix2NXd const control_points_gt{spline::Matrix2NXd::Random(6, 10)};

    database::InsertControlPoints(control_points_gt, CalibrationStep::SplineInterpolation, sensor_name, db);

    auto const control_points{database::ReadSplineControlPoints(db, CalibrationStep::SplineInterpolation, sensor_name)};
    EXPECT_TRUE(control_points.isApprox(control_points_gt));

    auto const unknown_sensor_data{
        database::ReadSplineControlPoints(db, CalibrationStep::SplineInterpolation, "/cam/retro/unknown")};
    EXPECT_EQ(unknown_sensor_data.cols(), 0);
}

TEST(DatabaseDatabaseRead, TestReadSplineTimeHandler) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    std::string_view sensor_name{"/cam/retro/123"};
    database::InsertStep(CalibrationStep::SplineInterpolation, "", sensor_name, db);
    spline::TimeHandler const time_handler_gt{100, 200};

    database::WriteToDb(time_handler_gt, CalibrationStep::SplineInterpolation, sensor_name, db);

    auto const time_handler{database::ReadSplineTimeHandler(db, CalibrationStep::SplineInterpolation, sensor_name)};
    ASSERT_TRUE(time_handler.has_value());
    EXPECT_EQ(time_handler, time_handler_gt);

    auto const unknown_sensor_data{
        database::ReadSplineTimeHandler(db, CalibrationStep::SplineInterpolation, "/cam/retro/unknown")};
    EXPECT_FALSE(unknown_sensor_data.has_value());
}

TEST(DatabaseDatabaseRead, TestReadExtrinsics) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    std::string_view sensor_name{"tf_imu_co"};
    database::InsertStep(CalibrationStep::ExtrinsicInitialization, "", sensor_name, db);
    Array6d const tf_imu_co_gt{0, 1, 2, 3, 4, 5};

    database::InsertExtrinsic(tf_imu_co_gt, CalibrationStep::ExtrinsicInitialization, sensor_name, db);

    auto const tf_imu_co{database::ReadExtrinsics(db, CalibrationStep::ExtrinsicInitialization, sensor_name)};
    ASSERT_TRUE(tf_imu_co.has_value());
    EXPECT_TRUE(tf_imu_co->isApprox(tf_imu_co_gt));

    auto const unknown_sensor_data{
        database::ReadExtrinsics(db, CalibrationStep::ExtrinsicInitialization, "tf_blah_blah")};
    EXPECT_FALSE(unknown_sensor_data.has_value());
}

TEST(DatabaseDatabaseRead, TestReadGravity) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    std::string_view sensor_name{"world"};
    database::InsertStep(CalibrationStep::ExtrinsicInitialization, "", sensor_name, db);
    Array3d const gravity_w_gt{0, 1, 2};

    database::InsertGravity(gravity_w_gt, CalibrationStep::ExtrinsicInitialization, sensor_name, db);

    auto const gravity_w{database::ReadGravity(db, CalibrationStep::ExtrinsicInitialization, sensor_name)};
    ASSERT_TRUE(gravity_w.has_value());
    EXPECT_TRUE(gravity_w->isApprox(gravity_w_gt));

    auto const unknown_sensor_data{database::ReadGravity(db, CalibrationStep::ExtrinsicInitialization, "gravity_blah")};
    EXPECT_FALSE(unknown_sensor_data.has_value());
}