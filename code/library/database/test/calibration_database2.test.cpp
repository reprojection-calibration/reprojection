#include "database/calibration_database2.hpp"

#include <gtest/gtest.h>

#include <string>

#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(Yyy, TestGetOrCreateAsset) {
    TemporaryFile const temp_file{".db3"};
    auto db{database::CalibrationDatabase(temp_file.Path(), true)};

    // Repeated insert returns the same id with no problems.
    database::AssetId result{db.GetOrCreateAsset(database::AssetType::Camera, 0, "/cam0/image_raw")};
    EXPECT_EQ(result, database::AssetId{1});
    result = db.GetOrCreateAsset(database::AssetType::Camera, 0, "/cam0/image_raw");
    EXPECT_EQ(result, database::AssetId{1});

    // Add a second camera at index 1, no problemo.
    result = db.GetOrCreateAsset(database::AssetType::Camera, 1, "/cam1/image_raw");
    EXPECT_EQ(result, database::AssetId{2});

    // Adding an imu with an already used index is also no problem,
    result = db.GetOrCreateAsset(database::AssetType::Imu, 0, "/imu0");
    EXPECT_EQ(result, database::AssetId{3});

    // Trying to insert an asset with a different name at an already existing index is a no-go!
    EXPECT_THROW(db.GetOrCreateAsset(database::AssetType::Camera, 0, "/cam1/image_raw"), std::runtime_error);
    // Trying to create a new asset with an already existing name is also a no-go!
    EXPECT_THROW(db.GetOrCreateAsset(database::AssetType::Camera, 2, "/cam1/image_raw"), database::SqliteException);
    // Even if the asset type changes you are still not allowed to reuse a name!
    EXPECT_THROW(db.GetOrCreateAsset(database::AssetType::Imu, 2, "/cam1/image_raw"), database::SqliteException);
}

TEST(Ggg, TestGetOrCreateRecording) {
    TemporaryFile const temp_file{".db3"};
    auto db{database::CalibrationDatabase(temp_file.Path(), true)};

    // Repeated insert with matching name and hash is no problem!
    database::RecordingId result{db.GetOrCreateRecording("recording.bag", "sha256-xxx")};
    EXPECT_EQ(result, database::RecordingId{1});
    result = db.GetOrCreateRecording("recording.bag", "sha256-xxx");
    EXPECT_EQ(result, database::RecordingId{1});

    // Adding another recording with a unique name and hash is no problem.
    result = db.GetOrCreateRecording("recording1.bag", "sha256-yyy");
    EXPECT_EQ(result, database::RecordingId{2});

    // Trying to insert an existing name with a different hash is a no-go!
    EXPECT_THROW(db.GetOrCreateRecording("recording1.bag", "sha256-zzz"), std::runtime_error);
    // Cannot insert two different recordings with the same hash.
    EXPECT_THROW(db.GetOrCreateRecording("recording2.bag", "sha256-yyy"), database::SqliteException);
}

TEST(Hhh, TestGetOrCreateRun) {
    TemporaryFile const temp_file{".db3"};
    auto db{database::CalibrationDatabase(temp_file.Path(), true)};

    // A run requires a recording - satisfy this foreign key constraint here.
    database::RecordingId const recording_id{db.GetOrCreateRecording("recording.bag", "sha256-xxx")};

    // Repeated insert is no problem.
    database::RunId result{db.GetOrCreateRun(recording_id, "[config]")};
    EXPECT_EQ(result, database::RunId{1});
    result = db.GetOrCreateRun(recording_id, "[config]");
    EXPECT_EQ(result, database::RunId{1});

    // Changing the config creates a new run.
    result = db.GetOrCreateRun(recording_id, "[config1]");
    EXPECT_EQ(result, database::RunId{2});

    // Trying to insert a run with a non-existent recording is an error.
    EXPECT_THROW(db.GetOrCreateRun(database::RecordingId{10}, "[config]"), database::SqliteException);
}

class GetOrCreateStepFixture : public ::testing::Test {
   protected:
    void SetUp() override { recording_id_ = db_.GetOrCreateRecording("recording.bag", "sha256-xxx"); }

    TemporaryFile temp_file_{".db3"};
    database::CalibrationDatabase db_{temp_file_.Path(), true};
    database::RecordingId recording_id_{-1};
};

TEST_F(GetOrCreateStepFixture, TestSingleOwnerSemantics) {
    // Insert a new step - cache miss.
    auto result{db_.GetOrCreateStep(recording_id_, std::nullopt, database::StepType::ImageLoading, "sha256-aaa")};
    EXPECT_EQ(result.first, database::StepId{1});
    EXPECT_EQ(result.second, false);

    // Attempting to insert the same step again does nothing - cache hit.
    result = db_.GetOrCreateStep(recording_id_, std::nullopt, database::StepType::ImageLoading, "sha256-aaa");
    EXPECT_EQ(result.first, database::StepId{1});
    EXPECT_EQ(result.second, true);

    // Inserting the same step but with a new cache key will delete the old entry and insert a new one with the same id
    // - cache miss.
    result = db_.GetOrCreateStep(recording_id_, std::nullopt, database::StepType::ImageLoading, "sha256-bbb");
    EXPECT_EQ(result.first, database::StepId{1});
    EXPECT_EQ(result.second, false);

    // Create a second recording.
    recording_id_ = db_.GetOrCreateRecording("recording1.bag", "sha256-yyy");

    // Inserting the same step under a new owner creates a new valid id - cache miss.
    result = db_.GetOrCreateStep(recording_id_, std::nullopt, database::StepType::ImageLoading, "sha256-bbb");
    EXPECT_EQ(result.first, database::StepId{2});
    EXPECT_EQ(result.second, false);

    // Creating a step under a non-existent recording is an error.
    EXPECT_THROW(
        db_.GetOrCreateStep(database::RecordingId{111}, std::nullopt, database::StepType::ImageLoading, "sha256-aaa"),
        database::SqliteException);
}

TEST_F(GetOrCreateStepFixture, TestDualOwnerSemantics) {
    // A step owned by a recording,
    auto result{db_.GetOrCreateStep(recording_id_, std::nullopt, database::StepType::ImageLoading, "sha256-aaa")};
    EXPECT_EQ(result.first, database::StepId{1});
    EXPECT_EQ(result.second, false);

    // Create a run.
    database::RunId const run_id{db_.GetOrCreateRun(recording_id_, "[config]")};

    // The same exact step but under the run's ownership creates a new step entry.
    result = db_.GetOrCreateStep(std::nullopt, run_id, database::StepType::ImageLoading, "sha256-aaa");
    EXPECT_EQ(result.first, database::StepId{2});
    EXPECT_EQ(result.second, false);

    // A step can only have one owner, either a recording or a run - therefore this is an error.
    EXPECT_THROW(db_.GetOrCreateStep(recording_id_, run_id, database::StepType::ImageLoading, "sha256-aaa"),
                 database::SqliteException);
}

TEST(Ddd, TestInsertImages) {
    TemporaryFile const temp_file{".db3"};
    auto db{database::CalibrationDatabase(temp_file.Path(), true)};

    // Satisfy the foreign key constraints. In words:
    //      In order to add images we need a dataset (recording_id) that has been processed (step_id) for a specific
    //      sensor (asset_id)
    //
    database::RecordingId const recording_id{db.GetOrCreateRecording("recording.bag", "sha256-xxx")};
    auto const step{db.GetOrCreateStep(recording_id, std::nullopt, database::StepType::ImageLoading, "sha256-bbb")};
    database::AssetId const asset_id{db.GetOrCreateAsset(database::AssetType::Camera, 0, "/cam0/image_raw")};

    EncodedImages const images{{0, ImageBuffer{}}};
    EXPECT_NO_THROW(db.InsertImages(step.first, asset_id, images));

    // Dual insertion is a violation of the uniqueness constraint.
    EXPECT_THROW(db.InsertImages(step.first, asset_id, images), database::SqliteException);

    // If step or asset ids are not valid this is an error.
    EXPECT_THROW(db.InsertImages(database::StepId{111}, asset_id, images), database::SqliteException);
    EXPECT_THROW(db.InsertImages(step.first, database::AssetId{111}, images), database::SqliteException);
}