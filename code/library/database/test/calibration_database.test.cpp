#include "database/calibration_database.hpp"

#include <gtest/gtest.h>

#include <string>

#include "database/sqlite_exception.hpp"
#include "types/database_types.hpp"

using namespace reprojection;

TEST(DatabaseCalibrationDatbase, TestGetOrCreateAsset) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    // Repeated insert returns the same id with no problems.
    AssetId result{db.GetOrCreateAsset(AssetType::Camera, 0, "/cam0/image_raw")};
    EXPECT_EQ(result, AssetId{1});
    result = db.GetOrCreateAsset(AssetType::Camera, 0, "/cam0/image_raw");
    EXPECT_EQ(result, AssetId{1});

    // Add a second camera at index 1, no problemo.
    result = db.GetOrCreateAsset(AssetType::Camera, 1, "/cam1/image_raw");
    EXPECT_EQ(result, AssetId{2});

    // Adding an imu with an already used index is also no problem,
    result = db.GetOrCreateAsset(AssetType::Imu, 0, "/imu0");
    EXPECT_EQ(result, AssetId{3});

    // Trying to insert an asset with a different name at an already existing index is a no-go!
    EXPECT_THROW(db.GetOrCreateAsset(AssetType::Camera, 0, "/cam1/image_raw"), std::runtime_error);
    // Trying to create a new asset with an already existing name is also a no-go!
    EXPECT_THROW(db.GetOrCreateAsset(AssetType::Camera, 2, "/cam1/image_raw"), database::SqliteException);
    // Even if the asset type changes you are still not allowed to reuse a name!
    EXPECT_THROW(db.GetOrCreateAsset(AssetType::Imu, 2, "/cam1/image_raw"), database::SqliteException);
}

TEST(DatabaseCalibrationDatbase, TestGetOrCreateRecording) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    // Repeated insert with matching name and hash is no problem!
    RecordingId result{db.GetOrCreateRecording("recording.bag", "sha256-xxx")};
    EXPECT_EQ(result, RecordingId{1});
    result = db.GetOrCreateRecording("recording.bag", "sha256-xxx");
    EXPECT_EQ(result, RecordingId{1});

    // Adding another recording with a unique name and hash is no problem.
    result = db.GetOrCreateRecording("recording1.bag", "sha256-yyy");
    EXPECT_EQ(result, RecordingId{2});

    // Trying to insert an existing name with a different hash is a no-go!
    EXPECT_THROW(db.GetOrCreateRecording("recording1.bag", "sha256-zzz"), std::runtime_error);
    // Cannot insert two different recordings with the same hash.
    EXPECT_THROW(db.GetOrCreateRecording("recording2.bag", "sha256-yyy"), database::SqliteException);
}

TEST(DatabaseCalibrationDatabase, TestGetOrCreateStep) {
    database::CalibrationDatabase db_{":memory:", true};
    Hash const hash_a{"sha256-aaa"};

    // Insert a new step - cache miss.
    auto result{db_.GetOrCreateStep(StepType::ImageLoading, hash_a)};
    EXPECT_EQ(result.first, StepId{1});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);

    // Until we "StepCacheKeyUpdate()" it will insert a new step and be a cache miss because there is no cache_key in
    // the database yet.
    result = db_.GetOrCreateStep(StepType::ImageLoading, hash_a);
    EXPECT_EQ(result.first, StepId{2});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);

    // Complete the second step entry with the cache key!
    db_.StepCacheKeyUpdate(result.first, hash_a);

    // Now that the cache key is there we get a cache hit.
    result = db_.GetOrCreateStep(StepType::ImageLoading, hash_a);
    EXPECT_EQ(result.first, StepId{2});
    EXPECT_EQ(result.second, CacheStatus::CacheHit);

    // Adding another ImageLoading step with a different cache key increments the step id and is a cache miss.
    Hash const hash_b{"sha256-bbb"};
    result = db_.GetOrCreateStep(StepType::ImageLoading, hash_b);
    EXPECT_EQ(result.first, StepId{3});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);
}

TEST(DatabaseCalibrationDatabase, TestCameraInfo) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    auto const step{db.GetOrCreateStep(StepType::CameraInfo, "")};
    AssetId const asset_id{db.GetOrCreateAsset(AssetType::Camera, 0, "")};

    CameraInfo const camera_info{CameraModel::DoubleSphere, {0, 512, 0, 512}};
    EXPECT_NO_THROW(db.CameraInfoInsert(step.first, asset_id, camera_info));

    auto result{db.CameraInfoSelect(step.first, asset_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->camera_model, camera_info.camera_model);
    EXPECT_EQ(result->bounds.u_max, camera_info.bounds.u_max);
    EXPECT_EQ(result->bounds.u_min, camera_info.bounds.u_min);

    EXPECT_NO_THROW(result = db.CameraInfoSelect(StepId{111}, asset_id));
    EXPECT_FALSE(result.has_value());
    EXPECT_NO_THROW(result = db.CameraInfoSelect(step.first, AssetId{111}));
    EXPECT_FALSE(result.has_value());
}

class CalibrationDatabaseFixture : public ::testing::Test {
   protected:
    void InsertImage(StepId const step_id, AssetId const asset_id, uint64_t const timestamp_ns = 0) {
        db_.ImagesInsert(step_id, asset_id, {{timestamp_ns, ImageBuffer{}}});
    }

    StepId CreateExtractedTargets(StepId const image_loading_id, AssetId const asset_id,
                                  uint64_t const timestamp_ns = 0) {
        auto const step_id{db_.GetOrCreateStep(StepType::FeatureExtraction, "").first};

        db_.ExtractedTargetsInsert(step_id, image_loading_id, asset_id, {{timestamp_ns, ExtractedTarget{}}});

        return step_id;
    }

    database::CalibrationDatabase db_{":memory:", true};
};

TEST_F(CalibrationDatabaseFixture, TestCameraPoses) {
    // Satisfy foreign key dependencies - a pose depends on a target which depends on an image.
    AssetId const asset_id{db_.GetOrCreateAsset(AssetType::Camera, 0, "")};
    StepId const image_loading_id{db_.GetOrCreateStep(StepType::ImageLoading, "").first};
    InsertImage(image_loading_id, asset_id);
    StepId const extracted_targets_id{CreateExtractedTargets(image_loading_id, asset_id)};

    Frames const camera_poses{Frame{0, Array6d::Ones(6)}};
    StepId const step_id{db_.GetOrCreateStep(StepType::PoseInit, "").first};
    EXPECT_NO_THROW(db_.CameraPosesInsert(step_id, extracted_targets_id, asset_id, camera_poses));

    auto const result{db_.CameraPosesSelect(step_id, asset_id)};
    EXPECT_EQ(std::size(result), 1);
    EXPECT_TRUE(result.at(0).pose.isApprox(camera_poses.at(0).pose));
}

TEST(DatabaseCalibrationDatbase, TestControlPoints) {
    database::CalibrationDatabase db{":memory:", true};

    StepId const step_id{db.GetOrCreateStep(StepType::SplineInit, "").first};
    AssetId const asset_id{db.GetOrCreateAsset(AssetType::Camera, 0, "")};

    spline::Matrix2NXd const control_points{spline::Matrix2NXd::Random(6, 3)};

    EXPECT_NO_THROW(db.ControlPointsInsert(step_id, asset_id, control_points));

    auto const result{db.ControlPointsSelect(step_id, asset_id)};
    EXPECT_TRUE(result.isApprox(control_points));
}

TEST_F(CalibrationDatabaseFixture, TestImages) {
    AssetId const asset_id{db_.GetOrCreateAsset(AssetType::Camera, 0, "")};
    StepId const step_id{db_.GetOrCreateStep(StepType::ImageLoading, "").first};

    uint64_t const timestamp_ns{0};
    EXPECT_NO_THROW(InsertImage(step_id, asset_id, timestamp_ns));

    auto const result{db_.ImagesSelect(step_id, asset_id)};
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(std::size(result.at(timestamp_ns).data), 0);
}

TEST(DatabaseCalibrationDatbase, TestImuData) {
    database::CalibrationDatabase db{":memory:", true};

    auto const step{db.GetOrCreateStep(StepType::ImuDataLoading, "")};
    AssetId const asset_id{db.GetOrCreateAsset(AssetType::Imu, 0, "")};

    ImuMeasurements const imu_data{{0, {{1, 2, 3}, {4, 5, 6}}}, {1, {{1, 2, 3}, {4, 5, 6}}}};

    // TODO(Jack): This is a weakness of the current database design and I am not sure if it can be solved. The problem
    // here for example is that we cannot enforce that the owner of the step is a recording, which for imu/image data is
    // basically a requirement (instead of being owned by a run). We also cannot ensure that the step type is the
    // ImuDataLoading type because we just pass in the step id here. These are not things which make the program
    // incorrect, but it does feel like we are leaving something on the table with respect to unused
    // information/constraints. Maybe we can use some post insertion consistency checks/logics to make sure the database
    // is consistent with these constraints after each operation.
    EXPECT_NO_THROW(db.ImuDataInsert(step.first, asset_id, imu_data));

    auto result{db.ImuDataSelect(step.first, asset_id)};
    EXPECT_EQ(std::size(result), std::size(imu_data));
    EXPECT_TRUE(result.at(0).angular_velocity.isApprox(imu_data.at(0).angular_velocity));

    // If nonexistent data is requested this is not an error, it will just return an empty container.
    EXPECT_NO_THROW(result = db.ImuDataSelect(StepId{111}, asset_id));
    EXPECT_EQ(std::size(result), 0);
    EXPECT_NO_THROW(result = db.ImuDataSelect(step.first, AssetId{111}));
    EXPECT_EQ(std::size(result), 0);
}

TEST(DatabaseCalibrationDatbase, TestIntrinsics) {
    database::CalibrationDatabase db{":memory:", true};

    // TODO(Jack): We absolutely need to add logic to the database that checks the owning step for a camera intrinsic is
    // valid (ex. comes from a step which produces an intrinsic) and that the asset id is a camera. We need this kind of
    // checks for all steps, not just here.
    auto const step{db.GetOrCreateStep(StepType::IntrinsicInit, "")};
    AssetId const asset_id{db.GetOrCreateAsset(AssetType::Camera, 0, "")};

    CameraState const intrinsic{Array3d{1, 2, 3}};

    EXPECT_NO_THROW(db.IntrinsicInsert(step.first, asset_id, CameraModel::Pinhole, intrinsic));

    auto result{db.IntrinsicSelect(step.first, asset_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->intrinsics.isApprox(intrinsic.intrinsics));

    EXPECT_NO_THROW(result = db.IntrinsicSelect(StepId{111}, asset_id));
    EXPECT_FALSE(result.has_value());
    EXPECT_NO_THROW(result = db.IntrinsicSelect(step.first, AssetId{111}));
    EXPECT_FALSE(result.has_value());
}

TEST_F(CalibrationDatabaseFixture, TestExtractedTargets) {
    // Satisfy foreign keys - a target requires a corresponding image to be present.
    AssetId const asset_id{db_.GetOrCreateAsset(AssetType::Camera, 0, "")};
    StepId const image_loading_id{db_.GetOrCreateStep(StepType::ImageLoading, "").first};
    InsertImage(image_loading_id, asset_id);

    StepId step_id;
    EXPECT_NO_THROW(step_id = CreateExtractedTargets(image_loading_id, asset_id));

    CameraMeasurements const result{db_.ExtractedTargetsSelect(step_id, asset_id)};
    EXPECT_EQ(std::size(result), 1);
    EXPECT_EQ(result.at(0).indices.size(), 0);
}

TEST(DatabaseCalibrationDatbase, TestExtrinsics) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    StepId const step_id{db.GetOrCreateStep(StepType::ExtrinsicInit, "").first};
    AssetId const cam_id{db.GetOrCreateAsset(AssetType::Camera, 0, "")};
    AssetId const imu_id{db.GetOrCreateAsset(AssetType::Imu, 0, "")};

    database::Extrinsic2 const extrinsic_co_imu{cam_id, imu_id, Array6d::Random()};
    EXPECT_NO_THROW(db.ExtrinsicInsert(step_id, extrinsic_co_imu));

    auto const result{db.ExtrinsicSelect(step_id, cam_id, imu_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->frame_a, cam_id);
    EXPECT_EQ(result->frame_b, imu_id);
    EXPECT_TRUE(result->se3_a_b.isApprox(extrinsic_co_imu.se3_a_b));
}

TEST(DatabaseCalibrationDatbase, TestSplineInfo) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    // Satisfy foreign key constraints
    StepId const step_id{db.GetOrCreateStep(StepType::SplineInit, "").first};
    AssetId const asset_id{db.GetOrCreateAsset(AssetType::Camera, 0, "")};

    spline::TimeHandler const time_handler{0, 1};
    EXPECT_NO_THROW(db.SplineInfoInsert(step_id, asset_id, time_handler));

    auto result{db.SplineInfoSelect(step_id, asset_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->t0_ns_, time_handler.t0_ns_);
    EXPECT_EQ(result->delta_t_ns_, time_handler.delta_t_ns_);
}

TEST(DatabaseCalibrationDatbase, TestTargetInfo) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    // Satisfy foreign key constraints
    auto const step{db.GetOrCreateStep(StepType::TargetInfo, "")};
    AssetId const asset_id{db.GetOrCreateAsset(AssetType::Target, 0, "")};

    TargetInfo const target_info{TargetType::Checkerboard, 6, 6, 0.1, false};
    EXPECT_NO_THROW(db.TargetInfoInsert(step.first, asset_id, target_info));

    auto result{db.TargetInfoSelect(step.first, asset_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, target_info.target_type);
    EXPECT_EQ(result->height, target_info.height);

    EXPECT_NO_THROW(result = db.TargetInfoSelect(StepId{111}, asset_id));
    EXPECT_FALSE(result.has_value());
    EXPECT_NO_THROW(result = db.TargetInfoSelect(step.first, AssetId{111}));
    EXPECT_FALSE(result.has_value());
}