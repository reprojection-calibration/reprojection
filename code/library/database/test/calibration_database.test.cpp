#include "database/calibration_database.hpp"

#include <gtest/gtest.h>

#include <string>

#include "database/sqlite_exception.hpp"
#include "types/database_types.hpp"

using namespace reprojection;

TEST(DatabaseCalibrationDatbase, TestOpenCalibrationDatabase) {
    // Cannot create a read only database because it requires writing to it to create it!
    EXPECT_THROW(auto db{database::OpenCalibrationDatabase(":memory:", true, true)}, std::runtime_error);

    // Happy paths
    EXPECT_NO_THROW(auto db{database::OpenCalibrationDatabase(":memory:", true)});
    EXPECT_NO_THROW(auto db{database::OpenCalibrationDatabase(":memory:", false, true)});
}

TEST(DatabaseCalibrationDatbase, TestGetOrCreateAsset) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    // Repeated insert returns the same id with no problems.
    AssetId result{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "/cam0/image_raw")};
    EXPECT_EQ(result, AssetId{1});
    result = database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "/cam0/image_raw");
    EXPECT_EQ(result, AssetId{1});

    // Add a second camera at index 1, no problemo.
    result = database::GetOrCreateAsset(db.get(), AssetType::Camera, 1, "/cam1/image_raw");
    EXPECT_EQ(result, AssetId{2});

    // Adding an imu with an already used index is also no problem,
    result = database::GetOrCreateAsset(db.get(), AssetType::Imu, 0, "/imu0");
    EXPECT_EQ(result, AssetId{3});

    // Trying to insert an asset with a different name at an already existing index is a no-go!
    EXPECT_THROW(database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "/cam1/image_raw"), std::runtime_error);
}

TEST(DatabaseCalibrationDatbase, TestGetOrCreateWorkflow) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};
    AssetId const asset_id_1{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};

    // Repeated insert with matching name and hash is no problem!
    WorkflowId result{database::GetOrCreateWorkflow(db.get(), WorkflowType::Cam, {asset_id_1})};
    EXPECT_EQ(result, WorkflowId{1});

    // Return the ID if you try to insert the same workflow again.
    result = database::GetOrCreateWorkflow(db.get(), WorkflowType::Cam, {asset_id_1});
    EXPECT_EQ(result, WorkflowId{1});

    // Inserting another workflow increments the workflow counter.
    AssetId const asset_id_2{database::GetOrCreateAsset(db.get(), AssetType::Camera, 1, "")};
    result = database::GetOrCreateWorkflow(db.get(), WorkflowType::CamImu, {asset_id_1, asset_id_2});
    EXPECT_EQ(result, WorkflowId{2});

    // Only one workflow entry is allowed for any single asset signature.
    EXPECT_THROW(database::GetOrCreateWorkflow(db.get(), WorkflowType::CamImu, {asset_id_1}),
                 database::SqliteException);
}

TEST(DatabaseCalibrationDatbase, TestAssetWorkflowState) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    AssetId cam_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};
    EXPECT_EQ(cam_id, AssetId{1});
    AssetId const target_id{database::GetOrCreateAsset(db.get(), AssetType::Target, 0, "")};
    EXPECT_EQ(target_id, AssetId{2});

    // Here we only add the target (should not happen in a real workflow), not the camera
    database::GetOrCreateWorkflow(db.get(), WorkflowType::Cam, {target_id});

    // The camera asset does not belong to any workflow so it gets deleted.
    database::DeleteUnusedAssets(db.get());

    // Insert the same exact camera asset that we did at the start - normally this would just return the original asset
    // with ID = 1, but because it got deleted in the previous step it actually creates a new asset now with ID = 3.
    cam_id = database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "");
    EXPECT_EQ(cam_id, AssetId{3});
}

TEST(DatabaseCalibrationDatbase, TestStepWorkflowState) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};
    database::AssetGroupInsert(db.get(), {asset_id});
    auto const workflow_id{database::GetOrCreateWorkflow(db.get(), WorkflowType::Cam, {asset_id})};

    // Create and finalize a step
    auto step_id{database::GetOrCreateStep(db.get(), StepType::CameraInfo, "").first};
    database::StepCacheKeyUpdate(db.get(), step_id, "");
    EXPECT_EQ(step_id, StepId{1});

    // Add that step to the workflow
    EXPECT_NO_THROW(database::WorkflowStepUpsert(db.get(), workflow_id, step_id, StepType::CameraInfo, {asset_id}));

    // Check that after adding it to the workflow its still there and with the same id (confirms it did not accidentally
    // get removed or something).
    step_id = database::GetOrCreateStep(db.get(), StepType::CameraInfo, "").first;
    EXPECT_EQ(step_id, StepId{1});

    // Now add another step of the same type, simulating running the calibration again but with a cache busted step.
    step_id = database::GetOrCreateStep(db.get(), StepType::CameraInfo, "x").first;
    database::StepCacheKeyUpdate(db.get(), step_id, "x");
    EXPECT_EQ(step_id, StepId{2});

    // Upsert the workflow now which removes the old step and replaces it with the new step id. Because of the "remove
    // unused steps" trigger the database has it should remove the first step.
    EXPECT_NO_THROW(database::WorkflowStepUpsert(db.get(), workflow_id, step_id, StepType::CameraInfo, {asset_id}));

    // If the first step hadn't been removed this should be a cache hit, but because it got removed this is a new step
    // with a new id.
    step_id = database::GetOrCreateStep(db.get(), StepType::CameraInfo, "").first;
    EXPECT_EQ(step_id, StepId{3});
}

TEST(DatabaseCalibrationDatabase, TestGetOrCreateStep) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};
    Hash const hash_a{"sha256-aaa"};

    // Insert a new step - cache miss.
    auto result{database::GetOrCreateStep(db.get(), StepType::ImageLoading, hash_a)};
    EXPECT_EQ(result.first, StepId{1});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);

    // Until we "database::StepCacheKeyUpdate()" it will insert a new step and be a cache miss because there is no
    // cache_key in the database yet.
    result = database::GetOrCreateStep(db.get(), StepType::ImageLoading, hash_a);
    EXPECT_EQ(result.first, StepId{2});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);

    // Complete the second step entry with the cache key!
    database::StepCacheKeyUpdate(db.get(), result.first, hash_a);

    // Now that the cache key is there we get a cache hit.
    result = database::GetOrCreateStep(db.get(), StepType::ImageLoading, hash_a);
    EXPECT_EQ(result.first, StepId{2});
    EXPECT_EQ(result.second, CacheStatus::CacheHit);

    // Adding another ImageLoading step with a different cache key increments the step id and is a cache miss.
    Hash const hash_b{"sha256-bbb"};
    result = database::GetOrCreateStep(db.get(), StepType::ImageLoading, hash_b);
    EXPECT_EQ(result.first, StepId{3});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);
}

TEST(DatabaseCalibrationDatabase, TestCameraInfo) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    auto const step{database::GetOrCreateStep(db.get(), StepType::CameraInfo, "")};
    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};

    CameraInfo const camera_info{CameraModel::DoubleSphere, {0, 512, 0, 512}};
    EXPECT_NO_THROW(database::CameraInfoInsert(db.get(), step.first, asset_id, camera_info));

    auto result{database::CameraInfoSelect(db.get(), step.first, asset_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->camera_model, camera_info.camera_model);
    EXPECT_EQ(result->bounds.u_max, camera_info.bounds.u_max);
    EXPECT_EQ(result->bounds.u_min, camera_info.bounds.u_min);

    EXPECT_NO_THROW(result = database::CameraInfoSelect(db.get(), StepId{111}, asset_id));
    EXPECT_FALSE(result.has_value());
    EXPECT_NO_THROW(result = database::CameraInfoSelect(db.get(), step.first, AssetId{111}));
    EXPECT_FALSE(result.has_value());

    // Query a nonexistent camera info so we can check the error message.
    result = database::CameraInfoSelect(db.get(), step.first, AssetId{-1});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), "{'database::': 'CameraInfoSelect', 'step_id': 1, 'asset_id': -1}");
}

class CalibrationDatabaseFixture : public ::testing::Test {
   protected:
    void InsertImage(StepId const step_id, AssetId const asset_id, uint64_t const timestamp_ns = 0) {
        database::ImagesInsert(db_.get(), step_id, asset_id, {{timestamp_ns, ImageBuffer{}}});
    }

    StepId CreateExtractedTargets(StepId const image_loading_id, AssetId const asset_id,
                                  uint64_t const timestamp_ns = 0) {
        auto const step_id{database::GetOrCreateStep(db_.get(), StepType::FeatureExtraction, "").first};

        database::ExtractedTargetsInsert(db_.get(), step_id, image_loading_id, asset_id,
                                         {{timestamp_ns, ExtractedTarget{}}});

        return step_id;
    }

    SqlitePtr db_{database::OpenCalibrationDatabase(":memory:", true)};
};

TEST_F(CalibrationDatabaseFixture, TestCameraPoses) {
    // Satisfy foreign key dependencies - a pose depends on a target which depends on an image.
    AssetId const asset_id{database::GetOrCreateAsset(db_.get(), AssetType::Camera, 0, "")};
    StepId const image_loading_id{database::GetOrCreateStep(db_.get(), StepType::ImageLoading, "").first};
    InsertImage(image_loading_id, asset_id);
    StepId const extracted_targets_id{CreateExtractedTargets(image_loading_id, asset_id)};

    Frames const camera_poses{Frame{0, Array6d::Ones(6)}};
    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::PoseInit, "").first};
    EXPECT_NO_THROW(database::CameraPosesInsert(db_.get(), step_id, extracted_targets_id, asset_id, camera_poses));

    auto const result{database::CameraPosesSelect(db_.get(), step_id, asset_id)};
    EXPECT_EQ(std::size(result), 1);
    EXPECT_TRUE(result.at(0).pose.isApprox(camera_poses.at(0).pose));
}

TEST(DatabaseCalibrationDatbase, TestControlPoints) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    StepId const step_id{database::GetOrCreateStep(db.get(), StepType::SplineInit, "").first};
    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};

    spline::Matrix2NXd const control_points{spline::Matrix2NXd::Random(6, 3)};

    EXPECT_NO_THROW(database::ControlPointsInsert(db.get(), step_id, asset_id, control_points));

    auto const result{database::ControlPointsSelect(db.get(), step_id, asset_id)};
    EXPECT_TRUE(result.isApprox(control_points));
}

TEST_F(CalibrationDatabaseFixture, TestImages) {
    AssetId const asset_id{database::GetOrCreateAsset(db_.get(), AssetType::Camera, 0, "")};
    StepId const step_id{database::GetOrCreateStep(db_.get(), StepType::ImageLoading, "").first};

    uint64_t const timestamp_ns{0};
    EXPECT_NO_THROW(InsertImage(step_id, asset_id, timestamp_ns));

    auto const result{database::ImagesSelect(db_.get(), step_id, asset_id)};
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::size(result.at(timestamp_ns).data), 0);
}

TEST(DatabaseCalibrationDatbase, TestImuData) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    StepId const imu_data_id{database::GetOrCreateStep(db.get(), StepType::ImuDataLoading, "").first};
    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Imu, 0, "")};

    ImuMeasurements const imu_data{{0, {{1, 2, 3}, {4, 5, 6}}}, {1, {{1, 2, 3}, {4, 5, 6}}}};

    EXPECT_NO_THROW(database::ImuDataInsert(db.get(), imu_data_id, asset_id, imu_data));

    auto const result{database::ImuDataSelect(db.get(), imu_data_id, asset_id)};
    EXPECT_EQ(std::size(result), std::size(imu_data));
    EXPECT_TRUE(result.at(0).angular_velocity.isApprox(imu_data.at(0).angular_velocity));
}

TEST(DatabaseCalibrationDatbase, TestImuErrors) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    StepId const imu_data_id{database::GetOrCreateStep(db.get(), StepType::ImuDataLoading, "").first};
    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Imu, 0, "")};

    ImuMeasurements const imu_data{{0, {{1, 2, 3}, {4, 5, 6}}}, {1, {{1, 2, 3}, {4, 5, 6}}}};
    database::ImuDataInsert(db.get(), imu_data_id, asset_id, imu_data);

    ImuErrors const imu_errors{{0, {{1, 2, 3}, {4, 5, 6}}}, {1, {{1, 2, 3}, {4, 5, 6}}}};

    StepId const extrinsic_init_id{database::GetOrCreateStep(db.get(), StepType::ExtrinsicInit, "").first};
    EXPECT_NO_THROW(database::ImuErrorsInsert(db.get(), extrinsic_init_id, imu_data_id, asset_id, imu_errors));
}

TEST(DatabaseCalibrationDatbase, TestIntrinsics) {
    auto const db{database::OpenCalibrationDatabase(":memory:", true)};

    // TODO(Jack): We absolutely need to add logic to the database that checks the owning step for a camera intrinsic is
    // valid (ex. comes from a step which produces an intrinsic) and that the asset id is a camera. We need this kind of
    // checks for all steps, not just here.
    auto const step{database::GetOrCreateStep(db.get(), StepType::IntrinsicInit, "")};
    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};

    CameraState const intrinsic{Array3d{1, 2, 3}};

    EXPECT_NO_THROW(database::IntrinsicInsert(db.get(), step.first, asset_id, CameraModel::Pinhole, intrinsic));

    auto result{database::IntrinsicSelect(db.get(), step.first, asset_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->intrinsics.isApprox(intrinsic.intrinsics));

    // Check error message.
    result = database::IntrinsicSelect(db.get(), step.first, AssetId{-1});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), "{'database::': 'IntrinsicSelect', 'step_id': 1, 'asset_id': -1}");
}

TEST_F(CalibrationDatabaseFixture, TestExtractedTargets) {
    // Satisfy foreign keys - a target requires a corresponding image to be present.
    AssetId const asset_id{database::GetOrCreateAsset(db_.get(), AssetType::Camera, 0, "")};
    StepId const image_loading_id{database::GetOrCreateStep(db_.get(), StepType::ImageLoading, "").first};
    InsertImage(image_loading_id, asset_id);

    StepId step_id;
    EXPECT_NO_THROW(step_id = CreateExtractedTargets(image_loading_id, asset_id));

    CameraMeasurements const result{database::ExtractedTargetsSelect(db_.get(), step_id, asset_id)};
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(result.at(0).indices.size(), 0);
}

TEST(DatabaseCalibrationDatbase, TestExtrinsics) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    StepId const step_id{database::GetOrCreateStep(db.get(), StepType::ExtrinsicInit, "").first};
    AssetId const cam_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};
    AssetId const imu_id{database::GetOrCreateAsset(db.get(), AssetType::Imu, 0, "")};

    Extrinsic const extrinsic_co_imu{cam_id, imu_id, Array6d::Random()};
    EXPECT_NO_THROW(database::ExtrinsicInsert(db.get(), step_id, extrinsic_co_imu));

    auto result{database::ExtrinsicSelect(db.get(), step_id, cam_id, imu_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->frame_a, cam_id);
    EXPECT_EQ(result->frame_b, imu_id);
    EXPECT_TRUE(result->se3_a_b.isApprox(extrinsic_co_imu.se3_a_b));

    // Check the error message.
    result = database::ExtrinsicSelect(db.get(), step_id, cam_id, AssetId{-1});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), "{'database::': 'ExtrinsicSelect', 'step_id': 1, 'asset_a_id': 1, 'asset_b_id': -1}");
}

TEST(DatabaseCalibrationDatbase, TestGravity) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    StepId const step_id{database::GetOrCreateStep(db.get(), StepType::ExtrinsicInit, "").first};

    Vector3d const gravity{Vector3d::Random()};
    EXPECT_NO_THROW(database::GravityInsert(db.get(), step_id, gravity));

    auto result{database::GravitySelect(db.get(), step_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->isApprox(gravity));

    // Check the error message.
    result = database::GravitySelect(db.get(), StepId{-1});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), "{'database::': 'GravitySelect', 'step_id': -1}");
}

TEST_F(CalibrationDatabaseFixture, TestReprojectionErrors) {
    // Use a common timestamp across image, target and reprojection error to make sure the foreign key constraint is
    // met.
    int const timestamp_ns{0};

    // Satisfy foreign keys - a target requires a corresponding image to be present.
    AssetId const asset_id{database::GetOrCreateAsset(db_.get(), AssetType::Camera, 0, "")};
    StepId const image_loading_id{database::GetOrCreateStep(db_.get(), StepType::ImageLoading, "").first};
    InsertImage(image_loading_id, asset_id, timestamp_ns);
    StepId const targets_id{CreateExtractedTargets(image_loading_id, asset_id, timestamp_ns)};

    ReprojectionErrors const data{{timestamp_ns, ArrayX2d{}}};

    StepId const reprojection_error_id{database::GetOrCreateStep(db_.get(), StepType::PoseInit, "").first};
    EXPECT_NO_THROW(database::ReprojectionErrorsInsert(db_.get(), reprojection_error_id, targets_id, asset_id, data));
}

TEST(DatabaseCalibrationDatbase, TestSplineInfo) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    // Satisfy foreign key constraints
    StepId const step_id{database::GetOrCreateStep(db.get(), StepType::SplineInit, "").first};
    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Camera, 0, "")};

    spline::TimeHandler const time_handler{0, 1};
    EXPECT_NO_THROW(database::SplineInfoInsert(db.get(), step_id, asset_id, time_handler));

    auto result{database::SplineInfoSelect(db.get(), step_id, asset_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->t0_ns_, time_handler.t0_ns_);
    EXPECT_EQ(result->delta_t_ns_, time_handler.delta_t_ns_);

    // Check the error message.
    result = database::SplineInfoSelect(db.get(), step_id, AssetId{-1});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), "{'database::': 'SplineInfoSelect', 'step_id': 1, 'asset_id': -1}");
}

TEST(DatabaseCalibrationDatbase, TestTargetInfo) {
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    // Satisfy foreign key constraints
    auto const step_id{database::GetOrCreateStep(db.get(), StepType::TargetInfo, "").first};
    AssetId const asset_id{database::GetOrCreateAsset(db.get(), AssetType::Target, 0, "")};

    TargetInfo const target_info{TargetType::Checkerboard, 6, 6, 0.1, false};
    EXPECT_NO_THROW(database::TargetInfoInsert(db.get(), step_id, asset_id, target_info));

    auto result{database::TargetInfoSelect(db.get(), step_id, asset_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, target_info.target_type);
    EXPECT_EQ(result->height, target_info.height);

    // Check the error message.
    result = database::TargetInfoSelect(db.get(), step_id, AssetId{-1});
    EXPECT_FALSE(result.has_value());
    EXPECT_EQ(result.error(), "{'database::': 'TargetInfoSelect', 'step_id': 1, 'asset_id': -1}");
}