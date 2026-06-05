#include "database/database_read.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "testing_utilities/constants.hpp"
#include "types/sensor_data_types.hpp"

#include "database_test_fixtures.hpp"

using namespace reprojection;

class CameraReadFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        db = database::OpenCalibrationDatabase(":memory:", true, false);

        database::InsertStep(db, sensor_name, CalibrationStep::CameraInfo, "");
        database::InsertCameraInfo(db, CameraInfo{sensor_name, CameraModel::Pinhole, testing_utilities::image_bounds});
    }

    void AddImage(uint64_t const timestamp_ns) const {
        // Due to foreign key relationship we need add an image before we add the target
        database::InsertStep(db, sensor_name, CalibrationStep::ImageLoading, "");
        database::InsertImages(db, sensor_name, EncodedImages{{timestamp_ns, {}}});
    }

    void AddTarget(uint64_t const timestamp_ns) const {
        // Due to foreign key relationship we need add an image before we add the target
        AddImage(timestamp_ns);

        database::InsertStep(db, sensor_name, CalibrationStep::FeatureExtraction, "");
        database::InsertTargets(db, sensor_name, {{timestamp_ns, target}});
    }

    SqlitePtr db{nullptr};
    std::string sensor_name{"/cam/retro/123"};
    ExtractedTarget target{Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}, {200.24, 300.56}},
                                  MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}, {300.65, 200.56, 712.57}}},
                           {{5, 6}, {2, 3}, {650, 600}}};
};

TEST_F(CameraDatabaseFixture, TestCameraInfo) {
    auto camera_info{database::ReadCameraInfo(db, "/nonexistent/camera")};
    EXPECT_FALSE(camera_info.has_value());

    EXPECT_NO_THROW(InsertCameraInfo());
    EXPECT_THROW(InsertCameraInfo(), std::runtime_error);  // Duplicate entry not allowed!

    camera_info = database::ReadCameraInfo(db, sensor_name);
    ASSERT_TRUE(camera_info.has_value());
    EXPECT_EQ(camera_info->sensor_name, sensor_name);
    EXPECT_EQ(camera_info->camera_model, CameraModel::Pinhole);
    EXPECT_EQ(camera_info->bounds.u_min, tu::image_bounds.u_min);
    EXPECT_EQ(camera_info->bounds.u_max, tu::image_bounds.u_max);
    EXPECT_EQ(camera_info->bounds.v_min, tu::image_bounds.v_min);
    EXPECT_EQ(camera_info->bounds.v_max, tu::image_bounds.v_max);
}

TEST_F(CameraDatabaseFixture, TestTargetInfo) {
    auto result{database::ReadTargetInfo(db, "/nonexistent/camera")};
    EXPECT_FALSE(result.has_value());

    EXPECT_NO_THROW(InsertTargetInfo());
    EXPECT_THROW(InsertTargetInfo(), std::runtime_error);

    result = database::ReadTargetInfo(db, sensor_name);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, target_info.target_type);
    EXPECT_EQ(result->height, target_info.height);
    EXPECT_EQ(result->width, target_info.width);
    EXPECT_EQ(result->unit_dimension, target_info.unit_dimension);
    EXPECT_EQ(result->asymmetric, target_info.asymmetric);
}

TEST_F(CameraDatabaseFixture, TestImages) {
    EncodedImages result{database::ReadImages(db, sensor_name)};
    EXPECT_EQ(std::size(result), 0);

    EXPECT_NO_THROW(InsertImage());
    EXPECT_THROW(InsertImage(), std::runtime_error);

    result = database::ReadImages(db, sensor_name);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns);

    // Loaded image is empty so the data array length is zero.
    EXPECT_EQ(std::size(result.at(timestamp_ns).data), 0);
}

TEST_F(CameraDatabaseFixture, TestIntrinsic) {
    auto const step_type{CalibrationStep::PoseInitialization};

    auto intrinsic{database::ReadIntrinsics(db, "/nonexistent/camera", step_type, camera_info.camera_model)};
    EXPECT_FALSE(intrinsic.has_value());

    EXPECT_THROW(InsertIntrinsic(step_type), std::runtime_error);

    // Satisfy foreign key requirements for a camera intrinsic
    InsertStep(step_type);
    InsertCameraInfo();

    EXPECT_NO_THROW(InsertIntrinsic(step_type));

    intrinsic = database::ReadIntrinsics(db, camera_info.sensor_name, step_type, camera_info.camera_model);
    ASSERT_TRUE(intrinsic.has_value());
    EXPECT_TRUE(intrinsic->isApprox(testing_utilities::pinhole_intrinsics));
}

TEST_F(CameraDatabaseFixture, TestPoses) {
    auto const step_type{CalibrationStep::PoseInitialization};

    Frames result{database::ReadPoses(db, sensor_name, step_type)};
    EXPECT_EQ(std::size(result), 0);

    EXPECT_THROW(InsertPose(step_type), std::runtime_error);

    // Satisfy foreign key constraints.
    InsertImage();
    InsertTarget();
    InsertStep(step_type);

    EXPECT_NO_THROW(InsertPose(step_type));

    result = database::ReadPoses(db, sensor_name, step_type);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns);

    EXPECT_TRUE(result.at(timestamp_ns).pose.isApprox(pose));
}

// NOTE(Jack): There is not "read" component of this test because as of now (05.06.2026) there is no reason or code to
// read reprojection errors back from the database.
TEST_F(CameraDatabaseFixture, TestReprojectionErrors) {
    std::map<uint64_t, ArrayX2d> const data{{timestamp_ns, ArrayX2d::Zero(1, 2)}};
    auto const step_type{CalibrationStep::PoseInitialization};

    EXPECT_THROW(database::InsertReprojectionErrors(db, sensor_name, step_type, data), std::runtime_error);

    // Satisfy foreign key constraints.
    InsertImage();
    InsertTarget();
    InsertStep(step_type);
    InsertPose(step_type);

    EXPECT_NO_THROW(database::InsertReprojectionErrors(db, sensor_name, step_type, data));
}

// NOTE(Jack): We do not use the test fixtures built in methods for the foreign key requirement check because the entity
// is already added in the SetUp() method. This test is also more complicated because we also test the ReadCacheKey()
// function which is essentially the read counterpart to the InsertStep() function.
TEST_F(CameraDatabaseFixture, TestStepAndReadCacheKey) {
    // Foreign key requirement is not satisfied
    EXPECT_THROW(db::InsertStep(db, "sensor_x", CalibrationStep::BundleAdjustment, "123"), std::runtime_error);

    auto result{db::ReadCacheKey(db, "sensor_x", CalibrationStep::BundleAdjustment)};
    EXPECT_FALSE(result.has_value());

    // Satisfy foreign key requirement.
    db::InsertEntity(db, "sensor_x", Entity::Camera);

    EXPECT_NO_THROW(db::InsertStep(db, "sensor_x", CalibrationStep::BundleAdjustment, "123"));

    result = db::ReadCacheKey(db, "sensor_x", CalibrationStep::BundleAdjustment);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, "123");

    // Finally check the "upsert" semantics for updating a given steps cache key. All the other insert methods throw
    // when you try to insert another value with the same primary key, but not the calibration step because we want the
    // ability to update the cache key during the calibration process.
    EXPECT_NO_THROW(InsertStep(CalibrationStep::PoseInitialization, "1"));
    EXPECT_NO_THROW(InsertStep(CalibrationStep::PoseInitialization, "2"));
    EXPECT_NO_THROW(InsertStep(CalibrationStep::PoseInitialization, "3"));
}

TEST_F(CameraDatabaseFixture, TestTargets) {
    CameraMeasurements result{database::ReadTargets(db, sensor_name)};
    EXPECT_EQ(std::size(result), 0);

    // Foreign key constraint not met.
    EXPECT_THROW(InsertTarget(), std::runtime_error);

    // Satisfy foreign key constraint.
    InsertImage();

    EXPECT_NO_THROW(InsertTarget());
    EXPECT_THROW(InsertTarget(), std::runtime_error);

    result = database::ReadTargets(db, sensor_name);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns);

    // Loaded target matches the groundtruth target that we wrote to the database at the start.
    auto const [bundle, indices]{result.at(timestamp_ns)};
    EXPECT_TRUE(bundle.pixels.isApprox(target.bundle.pixels));
    EXPECT_TRUE(bundle.points.isApprox(target.bundle.points));
    EXPECT_TRUE(indices.isApprox(target.indices));
}

TEST_F(ImuDatabaseFixture, TestImuMeasurements) {
    ImuMeasurements result{database::ReadImuData(db, sensor_name)};
    EXPECT_EQ(std::size(result), 0);

    // As of writing (05.06.2026) there are not yet any foreign key constraints for writing IMU dat into the db.
    EXPECT_NO_THROW(InsertImuData());

    result = database::ReadImuData(db, sensor_name);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns);

    auto const [omega, acc]{result.at(timestamp_ns)};
    EXPECT_TRUE(omega.isApprox(imu_data.at(timestamp_ns).angular_velocity));
    EXPECT_TRUE(acc.isApprox(imu_data.at(timestamp_ns).linear_acceleration));
}

TEST_F(ImuDatabaseFixture, TestImuErrors) {
    auto const step_type{CalibrationStep::ExtrinsicInitialization};

    ImuErrors result{database::ReadImuErrors(db, sensor_name, step_type)};
    EXPECT_EQ(std::size(result), 0);

    EXPECT_THROW(InsertImuError(step_type), std::runtime_error);

    // Satisfy foreign key constraints.
    InsertImuData();
    InsertStep(step_type);

    EXPECT_NO_THROW(InsertImuError(step_type));

    result = database::ReadImuErrors(db, sensor_name, step_type);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns);

    auto const [delta_omega, delta_acc]{result.at(timestamp_ns)};
    EXPECT_TRUE(delta_omega.isApprox(imu_errors.at(timestamp_ns).delta_angular_velocity));
    EXPECT_TRUE(delta_acc.isApprox(imu_errors.at(timestamp_ns).delta_linear_acceleration));
}

TEST_F(ExtrinsicDatabaseFixture, TestControlPoints) {
    auto const step_type{CalibrationStep::SplineInitialization};

    spline::Matrix2NXd result{database::ReadControlPoints(db, camera_name, step_type)};
    EXPECT_EQ(std::size(result), 0);

    spline::Matrix2NXd const data{spline::Matrix2NXd::Random(6, 10)};
    EXPECT_THROW(database::InsertControlPoints(db, camera_name, step_type, data), std::runtime_error);

    // Satisfy foreign key constraint.
    InsertStep(camera_name, step_type);

    EXPECT_NO_THROW(database::InsertControlPoints(db, camera_name, step_type, data));

    result = database::ReadControlPoints(db, camera_name, step_type);
    EXPECT_TRUE(result.isApprox(data));
}

TEST_F(ExtrinsicDatabaseFixture, TestReadTimeHandler) {
    auto const step_type{CalibrationStep::SplineInitialization};

    auto result{database::ReadTimeHandler(db, camera_name, step_type)};
    EXPECT_FALSE(result.has_value());

    spline::TimeHandler const data{100, 200};
    EXPECT_THROW(database::InsertTimeHandler(db, camera_name, step_type, data), std::runtime_error);

    // Satisfy foreign key constraint.
    InsertStep(camera_name, step_type);

    EXPECT_NO_THROW(database::InsertTimeHandler(db, camera_name, step_type, data));

    result = database::ReadTimeHandler(db, camera_name, step_type);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result, data);
}

TEST(DatabaseDatabaseRead, TestReadExtrinsics) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    std::string_view sensor_name{"tf_imu_co"};
    database::InsertStep(db, sensor_name, CalibrationStep::ExtrinsicInitialization, "");
    Array6d const tf_imu_co_gt{0, 1, 2, 3, 4, 5};

    database::InsertExtrinsic(db, sensor_name, CalibrationStep::ExtrinsicInitialization, tf_imu_co_gt);

    auto const tf_imu_co{database::ReadExtrinsics(db, sensor_name, CalibrationStep::ExtrinsicInitialization)};
    ASSERT_TRUE(tf_imu_co.has_value());
    EXPECT_TRUE(tf_imu_co->isApprox(tf_imu_co_gt));

    auto const unknown_sensor_data{
        database::ReadExtrinsics(db, "tf_blah_blah", CalibrationStep::ExtrinsicInitialization)};
    EXPECT_FALSE(unknown_sensor_data.has_value());
}

TEST(DatabaseDatabaseRead, TestReadGravity) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    std::string_view sensor_name{"world"};
    database::InsertStep(db, sensor_name, CalibrationStep::ExtrinsicInitialization, "");
    Array3d const gravity_w_gt{0, 1, 2};

    database::InsertGravity(db, sensor_name, CalibrationStep::ExtrinsicInitialization, gravity_w_gt);

    auto const gravity_w{database::ReadGravity(db, sensor_name, CalibrationStep::ExtrinsicInitialization)};
    ASSERT_TRUE(gravity_w.has_value());
    EXPECT_TRUE(gravity_w->isApprox(gravity_w_gt));

    auto const unknown_sensor_data{database::ReadGravity(db, "gravity_blah", CalibrationStep::ExtrinsicInitialization)};
    EXPECT_FALSE(unknown_sensor_data.has_value());
}