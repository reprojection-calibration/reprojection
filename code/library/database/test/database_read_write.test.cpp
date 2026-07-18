#include <gtest/gtest.h>

#include <filesystem>
#include <map>
#include <string>

#include "database/calibration_database.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "testing_utilities/constants.hpp"
#include "types/sensor_data_types.hpp"

#include "database_test_fixtures.hpp"

using namespace reprojection;

TEST_F(CameraDatabaseFixture, TestCameraInfo) {
    auto camera_info{database::ReadCameraInfo(db_, "/nonexistent/camera")};
    EXPECT_FALSE(camera_info.has_value());

    EXPECT_NO_THROW(InsertCameraInfo());
    EXPECT_THROW(InsertCameraInfo(), std::runtime_error);  // Duplicate entry not allowed!

    camera_info = database::ReadCameraInfo(db_, sensor_name_);
    ASSERT_TRUE(camera_info.has_value());
    EXPECT_EQ(camera_info->sensor_name, sensor_name_);
    EXPECT_EQ(camera_info->camera_model, CameraModel::Pinhole);
    EXPECT_EQ(camera_info->bounds.u_min, tu::image_bounds.u_min);
    EXPECT_EQ(camera_info->bounds.u_max, tu::image_bounds.u_max);
    EXPECT_EQ(camera_info->bounds.v_min, tu::image_bounds.v_min);
    EXPECT_EQ(camera_info->bounds.v_max, tu::image_bounds.v_max);
}

TEST_F(CameraDatabaseFixture, TestTargetInfo) {
    auto result{database::ReadTargetInfo(db_, "/nonexistent/camera")};
    EXPECT_FALSE(result.has_value());

    EXPECT_NO_THROW(InsertTargetInfo());
    EXPECT_THROW(InsertTargetInfo(), std::runtime_error);

    result = database::ReadTargetInfo(db_, sensor_name_);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, target_info_.target_type);
    EXPECT_EQ(result->height, target_info_.height);
    EXPECT_EQ(result->width, target_info_.width);
    EXPECT_EQ(result->unit_dimension, target_info_.unit_dimension);
    EXPECT_EQ(result->asymmetric, target_info_.asymmetric);
}

TEST_F(CameraDatabaseFixture, TestImages) {
    EncodedImages result{database::ReadImages(db_, sensor_name_)};
    EXPECT_EQ(std::size(result), 0);

    EXPECT_NO_THROW(InsertImage());
    EXPECT_THROW(InsertImage(), std::runtime_error);

    result = database::ReadImages(db_, sensor_name_);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns_);

    // Loaded image is empty so the data array length is zero.
    EXPECT_EQ(std::size(result.at(timestamp_ns_).data), 0);
}

TEST_F(CameraDatabaseFixture, TestIntrinsic) {
    auto const step_type{CalibrationStep::PoseInitialization};

    auto intrinsic{database::ReadIntrinsics(db_, "/nonexistent/camera", step_type, camera_info_.camera_model)};
    EXPECT_FALSE(intrinsic.has_value());

    EXPECT_THROW(InsertIntrinsic(step_type), std::runtime_error);

    // Satisfy foreign key requirements for a camera intrinsic
    InsertStep(step_type);
    InsertCameraInfo();
    EXPECT_NO_THROW(InsertIntrinsic(step_type));

    intrinsic = database::ReadIntrinsics(db_, camera_info_.sensor_name, step_type, camera_info_.camera_model);
    ASSERT_TRUE(intrinsic.has_value());
    EXPECT_TRUE(intrinsic->isApprox(testing_utilities::pinhole_intrinsics));
}

TEST_F(CameraDatabaseFixture, TestPoses) {
    auto const step_type{CalibrationStep::PoseInitialization};

    Frames result{database::ReadPoses(db_, sensor_name_, step_type)};
    EXPECT_EQ(std::size(result), 0);

    EXPECT_THROW(InsertPose(step_type), std::runtime_error);

    // Satisfy foreign key constraints.
    InsertImage();
    InsertTarget();
    InsertStep(step_type);
    EXPECT_NO_THROW(InsertPose(step_type));

    result = database::ReadPoses(db_, sensor_name_, step_type);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns_);

    EXPECT_TRUE(result.at(timestamp_ns_).pose.isApprox(pose_));
}

// NOTE(Jack): There is not "read" component of this test because as of now (05.06.2026) there is no reason or code to
// read reprojection errors back from the database.
TEST_F(CameraDatabaseFixture, TestReprojectionErrors) {
    auto const step_type{CalibrationStep::PoseInitialization};

    std::map<uint64_t, ArrayX2d> const data{{timestamp_ns_, ArrayX2d::Zero(1, 2)}};
    EXPECT_THROW(database::InsertReprojectionErrors(db_, sensor_name_, step_type, data), std::runtime_error);

    // Satisfy foreign key constraints.
    InsertImage();
    InsertTarget();
    InsertStep(step_type);
    InsertPose(step_type);
    EXPECT_NO_THROW(database::InsertReprojectionErrors(db_, sensor_name_, step_type, data));
}

// NOTE(Jack): We do not use the test fixtures built in methods for the foreign key requirement check because the entity
// is already added in the SetUp() method. This test is also more complicated because we also test the ReadHashInputs()
// function which is essentially the read counterpart to the InsertStep() function.
TEST_F(CameraDatabaseFixture, TestStepAndReadCacheKey) {
    // Foreign key requirement is not satisfied
    EXPECT_THROW(db::InsertStep(db_, "sensor_x", CalibrationStep::BundleAdjustment, "123"), std::runtime_error);

    auto result{db::ReadCacheKey(db_, "sensor_x", CalibrationStep::BundleAdjustment)};
    EXPECT_FALSE(result.has_value());

    // Satisfy foreign key requirement.
    db::InsertEntity(db_, "sensor_x", Entity::Camera);
    EXPECT_NO_THROW(db::InsertStep(db_, "sensor_x", CalibrationStep::BundleAdjustment, "123"));

    result = db::ReadCacheKey(db_, "sensor_x", CalibrationStep::BundleAdjustment);
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
    CameraMeasurements result{database::ReadTargets(db_, sensor_name_)};
    EXPECT_EQ(std::size(result), 0);

    // Foreign key constraint not met.
    EXPECT_THROW(InsertTarget(), std::runtime_error);

    // Satisfy foreign key constraint.
    InsertImage();
    EXPECT_NO_THROW(InsertTarget());
    EXPECT_THROW(InsertTarget(), std::runtime_error);

    result = database::ReadTargets(db_, sensor_name_);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns_);

    // Loaded target matches the groundtruth target that we wrote to the database at the start.
    auto const [bundle, indices]{result.at(timestamp_ns_)};
    EXPECT_TRUE(bundle.pixels.isApprox(target_.bundle.pixels));
    EXPECT_TRUE(bundle.points.isApprox(target_.bundle.points));
    EXPECT_TRUE(indices.isApprox(target_.indices));
}

TEST_F(ImuDatabaseFixture, TestImuMeasurements) {
    ImuMeasurements result{database::ReadImuData(db_, imu_name_)};
    EXPECT_EQ(std::size(result), 0);

    // As of writing (05.06.2026) there are not yet any foreign key constraints for writing IMU dat into the db.
    EXPECT_NO_THROW(InsertImuData());

    result = database::ReadImuData(db_, imu_name_);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns_);

    auto const [omega, acc]{result.at(timestamp_ns_)};
    EXPECT_TRUE(omega.isApprox(imu_data_.at(timestamp_ns_).angular_velocity));
    EXPECT_TRUE(acc.isApprox(imu_data_.at(timestamp_ns_).linear_acceleration));
}

TEST_F(ImuDatabaseFixture, TestImuErrors) {
    auto const step_type{CalibrationStep::ExtrinsicInitialization};

    ImuErrors result{database::ReadImuErrors(db_, imu_name_, step_type)};
    EXPECT_EQ(std::size(result), 0);

    EXPECT_THROW(InsertImuError(step_type), std::runtime_error);

    // Satisfy foreign key constraints.
    InsertImuData();
    EXPECT_NO_THROW(InsertImuError(step_type));

    result = database::ReadImuErrors(db_, extrinsic_id_, step_type);
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::cbegin(result)->first, timestamp_ns_);

    auto const [delta_omega, delta_acc]{result.at(timestamp_ns_)};
    EXPECT_TRUE(delta_omega.isApprox(imu_errors_.at(timestamp_ns_).delta_angular_velocity));
    EXPECT_TRUE(delta_acc.isApprox(imu_errors_.at(timestamp_ns_).delta_linear_acceleration));
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

TEST_F(ExtrinsicDatabaseFixture, TestExtrinsics) {
    auto const step_type{CalibrationStep::SplineInitialization};

    auto result{database::ReadExtrinsics(db, extrinsic_id, step_type)};
    EXPECT_FALSE(result.has_value());

    Extrinsic const data{imu_name, camera_name, Array6d{0, 1, 2, 3, 4, 5}};
    EXPECT_THROW(database::InsertExtrinsic(db, extrinsic_id, step_type, data), std::runtime_error);

    // Satisfy foreign key constraint.
    InsertStep(extrinsic_id, step_type);
    EXPECT_NO_THROW(database::InsertExtrinsic(db, extrinsic_id, step_type, data));

    result = database::ReadExtrinsics(db, extrinsic_id, step_type);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->frame_a, data.frame_a);
    EXPECT_EQ(result->frame_b, data.frame_b);
    EXPECT_TRUE(result->se3_a_b.isApprox(data.se3_a_b));
}

TEST_F(ExtrinsicDatabaseFixture, TestGravity) {
    auto const step_type{CalibrationStep::SplineInitialization};

    auto result{database::ReadGravity(db, extrinsic_id, step_type)};
    EXPECT_FALSE(result.has_value());

    Array3d const data{0, 1, 2};
    EXPECT_THROW(database::InsertGravity(db, camera_name, step_type, data), std::runtime_error);

    // Satisfy foreign key constraint.
    InsertStep(camera_name, step_type);
    EXPECT_NO_THROW(database::InsertGravity(db, camera_name, step_type, data));

    result = database::ReadGravity(db, camera_name, step_type);
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->isApprox(data));
}

TEST_F(ExtrinsicDatabaseFixture, TestTimeHandler) {
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
