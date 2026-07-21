#include "database/calibration_database.hpp"

#include <gtest/gtest.h>

#include <string>

#include "database/sqlite_exception.hpp"
#include "types/database_types.hpp"

using namespace reprojection;

TEST(Yyy, TestGetOrCreateAsset) {
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

TEST(Ggg, TestGetOrCreateRecording) {
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

TEST(Hhh, TestGetOrCreateRun) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    // A run requires a recording - satisfy this foreign key constraint here.
    RecordingId const recording_id{db.GetOrCreateRecording("recording.bag", "sha256-xxx")};

    // Repeated insert is no problem.
    RunId result{db.GetOrCreateRun(recording_id, "[config]")};
    EXPECT_EQ(result, RunId{1});
    result = db.GetOrCreateRun(recording_id, "[config]");
    EXPECT_EQ(result, RunId{1});

    // Changing the config creates a new run.
    result = db.GetOrCreateRun(recording_id, "[config1]");
    EXPECT_EQ(result, RunId{2});

    // Trying to insert a run with a non-existent recording is an error.
    EXPECT_THROW(db.GetOrCreateRun(RecordingId{10}, "[config]"), database::SqliteException);
}

class GetOrCreateStepFixture : public ::testing::Test {
   protected:
    void SetUp() override { recording_id_ = db_.GetOrCreateRecording("recording.bag", "sha256-xxx"); }

    database::CalibrationDatabase db_{":memory:", true};
    RecordingId recording_id_{-1};
};

TEST_F(GetOrCreateStepFixture, TestSingleOwnerSemantics) {
    // Insert a new step - cache miss.
    auto result{db_.GetOrCreateStep(recording_id_, std::nullopt, StepType::ImageLoading, "sha256-aaa")};
    EXPECT_EQ(result.first, StepId{1});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);

    // Attempting to insert the same step again does nothing - cache hit.
    result = db_.GetOrCreateStep(recording_id_, std::nullopt, StepType::ImageLoading, "sha256-aaa");
    EXPECT_EQ(result.first, StepId{1});
    EXPECT_EQ(result.second, CacheStatus::CacheHit);

    // Inserting the same step but with a new cache key will delete the old entry and insert a new one with the same id
    // - cache miss.
    result = db_.GetOrCreateStep(recording_id_, std::nullopt, StepType::ImageLoading, "sha256-bbb");
    EXPECT_EQ(result.first, StepId{1});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);

    // Create a second recording.
    recording_id_ = db_.GetOrCreateRecording("recording1.bag", "sha256-yyy");

    // Inserting the same step under a new owner creates a new valid id - cache miss.
    result = db_.GetOrCreateStep(recording_id_, std::nullopt, StepType::ImageLoading, "sha256-bbb");
    EXPECT_EQ(result.first, StepId{2});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);

    // Creating a step under a non-existent recording is an error.
    EXPECT_THROW(db_.GetOrCreateStep(RecordingId{111}, std::nullopt, StepType::ImageLoading, "sha256-aaa"),
                 database::SqliteException);
}

TEST_F(GetOrCreateStepFixture, TestDualOwnerSemantics) {
    // A step owned by a recording,
    auto result{db_.GetOrCreateStep(recording_id_, std::nullopt, StepType::ImageLoading, "sha256-aaa")};
    EXPECT_EQ(result.first, StepId{1});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);

    // Create a run.
    RunId const run_id{db_.GetOrCreateRun(recording_id_, "[config]")};

    // The same exact step but under the run's ownership creates a new step entry.
    result = db_.GetOrCreateStep(std::nullopt, run_id, StepType::ImageLoading, "sha256-aaa");
    EXPECT_EQ(result.first, StepId{2});
    EXPECT_EQ(result.second, CacheStatus::CacheMiss);

    // A step can only have one owner, either a recording or a run - therefore this is an error.
    EXPECT_THROW(db_.GetOrCreateStep(recording_id_, run_id, StepType::ImageLoading, "sha256-aaa"),
                 database::SqliteException);
}

class ImagesFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        recording_id_ = db_.GetOrCreateRecording("recording.bag", "sha256-xxx");
        auto const step{db_.GetOrCreateStep(recording_id_, std::nullopt, StepType::ImageLoading, "sha256-bbb")};
        step_id_ = step.first;
        asset_id_ = db_.GetOrCreateAsset(AssetType::Camera, 0, "/cam0/image_raw");
    }

    database::CalibrationDatabase db_{":memory:", true};
    RecordingId recording_id_{-1};
    StepId step_id_{-1};
    AssetId asset_id_{-1};
};

TEST_F(ImagesFixture, TestImagesInsert) {
    EncodedImages const images{{0, ImageBuffer{}}};
    EXPECT_NO_THROW(db_.ImagesInsert(step_id_, asset_id_, images));

    // Dual insertion is a violation of the uniqueness constraint.
    EXPECT_THROW(db_.ImagesInsert(step_id_, asset_id_, images), database::SqliteException);

    // If step or asset ids are not valid this is an error.
    EXPECT_THROW(db_.ImagesInsert(StepId{111}, asset_id_, images), database::SqliteException);
    EXPECT_THROW(db_.ImagesInsert(step_id_, AssetId{111}, images), database::SqliteException);
}

TEST_F(ImagesFixture, TestImagesSelect) {
    // We need first insert an image so we habe an image to select :)
    EncodedImages const images{{0, ImageBuffer{}}};
    db_.ImagesInsert(step_id_, asset_id_, images);

    EncodedImages result{db_.ImagesSelect(step_id_, asset_id_)};
    EXPECT_EQ(std::size(result), std::size(images));
    EXPECT_EQ(std::size(result.at(0).data), std::size(images.at(0).data));

    // If nonexistent data is requested this is not an error, it will just return an empty container.
    EXPECT_NO_THROW(result = db_.ImagesSelect(StepId{111}, asset_id_));
    EXPECT_EQ(std::size(result), 0);
    EXPECT_NO_THROW(result = db_.ImagesSelect(step_id_, AssetId{111}));
    EXPECT_EQ(std::size(result), 0);
}

class ExtractedTargetsFixture : public ::testing::Test {
   protected:
    void SetUp() override {
        RecordingId const recording_id{db_.GetOrCreateRecording("recording.bag", "sha256-xxx")};
        asset_id_ = db_.GetOrCreateAsset(AssetType::Camera, 0, "/cam0/image_raw");

        auto step{db_.GetOrCreateStep(recording_id, std::nullopt, StepType::ImageLoading, "sha256-bbb")};
        image_loading_step_id_ = step.first;
        // Each extracted target required a correspondent image
        EncodedImages const images{{0, ImageBuffer{}}};
        db_.ImagesInsert(step.first, asset_id_, images);

        step = db_.GetOrCreateStep(recording_id, std::nullopt, StepType::FeatureExtraction, "sha256-ccc");
        extracted_targets_step_id_ = step.first;
    }

    database::CalibrationDatabase db_{":memory:", true};
    AssetId asset_id_{-1};
    StepId image_loading_step_id_{-1};
    StepId extracted_targets_step_id_{-1};
};

TEST_F(ExtractedTargetsFixture, TestExtractedTargetsInsert) {
    CameraMeasurements extracted_targets{{0, ExtractedTarget{}}};
    EXPECT_NO_THROW(
        db_.ExtractedTargetsInsert(extracted_targets_step_id_, image_loading_step_id_, asset_id_, extracted_targets));

    // If the current step or the source step does not exist it's an error.
    EXPECT_THROW(db_.ExtractedTargetsInsert(extracted_targets_step_id_, StepId{111}, asset_id_, extracted_targets),
                 database::SqliteException);
    EXPECT_THROW(db_.ExtractedTargetsInsert(StepId{111}, image_loading_step_id_, asset_id_, extracted_targets),
                 database::SqliteException);

    // If there is not matching image we will throw.
    extracted_targets = CameraMeasurements{{1, ExtractedTarget{}}};
    EXPECT_THROW(
        db_.ExtractedTargetsInsert(extracted_targets_step_id_, image_loading_step_id_, asset_id_, extracted_targets),
        database::SqliteException);
}

TEST_F(ExtractedTargetsFixture, TestExtractedTargetsSelect) {
    // Insert a target so we have a target to read :)
    CameraMeasurements extracted_targets{{0, ExtractedTarget{}}};
    db_.ExtractedTargetsInsert(extracted_targets_step_id_, image_loading_step_id_, asset_id_, extracted_targets);

    CameraMeasurements result{db_.ExtractedTargetsSelect(extracted_targets_step_id_, asset_id_)};
    EXPECT_EQ(std::size(result), std::size(extracted_targets));
    EXPECT_EQ(result.at(0).indices.size(), extracted_targets.at(0).indices.size());

    // If nonexistent data is requested this is not an error, it will just return an empty container.
    EXPECT_NO_THROW(result = db_.ExtractedTargetsSelect(StepId{111}, asset_id_));
    EXPECT_EQ(std::size(result), 0);
    EXPECT_NO_THROW(result = db_.ExtractedTargetsSelect(extracted_targets_step_id_, AssetId{111}));
    EXPECT_EQ(std::size(result), 0);
}

TEST(Qqq, TestTargetInfo) {
    auto db{database::CalibrationDatabase(":memory:", true)};

    // Satisfy foreign key constraints
    RecordingId const recording_id{db.GetOrCreateRecording("recording.bag", "")};
    RunId const run_id{db.GetOrCreateRun(recording_id, "")};
    // NOTE(Jack): Target info is associated with the configuration file which is why we make it a child of the run_id
    // and not the recording_id.
    auto const step{db.GetOrCreateStep(std::nullopt, run_id, StepType::TargetInfo, "")};
    AssetId const asset_id{db.GetOrCreateAsset(AssetType::Target, 0, "")};

    TargetInfo const target_info{TargetType::Checkerboard, 6, 6, 0.1, 0};
    EXPECT_NO_THROW(db.TargetInfoInsert(step.first, asset_id, target_info));

    auto result{db.TargetInfoSelect(step.first, asset_id)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->target_type, target_info.target_type);
    EXPECT_EQ(result->height, target_info.height);

    // If nonexistent data is requested this is not an error, it will just return an empty container.
    EXPECT_NO_THROW(result = db.TargetInfoSelect(StepId{111}, asset_id));
    EXPECT_FALSE(result.has_value());
    EXPECT_NO_THROW(result = db.TargetInfoSelect(step.first, AssetId{111}));
    EXPECT_FALSE(result.has_value());
}