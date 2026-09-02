#include "application/reprojection_calibration.hpp"

#include <gtest/gtest.h>

#include <memory>

#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "steps/initialize_workflow.hpp"
#include "testing_utilities/database_setup_utils.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/calibration_config.hpp"
#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ApplicationReprojectionCalibration, TestParseArgs) {
    auto result{application::ParseArgs(1, nullptr)};
    EXPECT_FALSE(result.has_value());

    TemporaryFile const config_file{".toml", testing_utilities::calibration_config};

    char const arg0[]{"program"};
    char const arg1[]{"--config"};
    std::string const arg2{config_file.Path().string()};
    char const arg3[]{"--data"};
    // TODO(Jack): We are implicitly relying on the fact that this directory exists because it is the folder where the
    // TemporaryFile gets created by the fs::temp_directory_path() call. This is ts=a little hacky and it might cause us
    // problems if the assumption turns out not to be true.
    char const arg4[]{"/tmp"};
    char const arg5[]{"--workspace"};
    char const arg6[]{"/tmp"};
    char const* const argv[]{arg0, arg1, arg2.c_str(), arg3, arg4, arg5, arg6};

    int const argc{7};
    result = application::ParseArgs(argc, argv);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->data_path, "/tmp");  // Heuristic check of one of the values
}

TEST(ApplicationReprojectionCalibration, TestParseSensors) {
    toml::table const config{toml::parse(testing_utilities::calibration_config)};

    application::Sensors const sensors{application::ParseSensors(config)};

    EXPECT_EQ(std::size(sensors.camera_names), 2);
    for (size_t i{0}; i < std::size(sensors.camera_names); ++i) {
        auto const& cam_i{sensors.camera_names[i]};
        EXPECT_EQ(cam_i, std::format("/cam{}/image_raw", i));
    }

    ASSERT_TRUE(sensors.imu_name.has_value());
    EXPECT_EQ(*sensors.imu_name, "/imu0");
}

// WARN(Jack): I would really really like to also be able to exercise the imu calibration component here but it
// is not nearly as easy to generate cache hits for those steps with empty inputs/outputs. This requires some
// more investigation and until then we just need pass std::nullopt for the imu input. NOTE(Jack): We do not
// need to do anything for the pose_initialization and bundle_adjustment steps to manufacture a cache hit
// because if their inputs are empty they themselves will just pass through with no problem. This might change
// in the future but for now it stands.
// TODO(Jack): This test is a little sketchy because we are trying to induce cache hits to avoid actually having to
// calculate anything. As a principle we do not want to use the checked in test database which means this is as much
// as we can do here. I guess we could also use the MVG test data generator, but that will be for a future
// contributor :)
TEST(ApplicationReprojectionCalibration, TestCalibrate) {
    toml::table const config{toml::parse(testing_utilities::calibration_config)};
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    steps::CalibrationContext const context{steps::InitializeCalibration(config, db)};

    // TODO(Jack): Use a vector so we do not need to hardcode this to an array matching the number of cameras in the
    // config?
    std::vector<testing_utilities::CameraTestData> camera_test_data;
    for (size_t i{0}; i < std::size(context.assets.cameras); ++i) {
        auto const& camera{context.assets.cameras.at(i)};

        // NOTE(Jack): We need to simulate some cache keys here because the database has UNIQUE constraints that prevent
        // us from just entering a blank string or something like that.
        Hash const image_cache_key{std::format("img{}", i)};
        Hash const feature_cache_key{std::format("ftex{}", i)};

        // WARN(Jack): If the camera info cache key calculation method changes then we will need to update this here
        // too (specifically the call to HashArguments())!
        camera_test_data.push_back({
            database::GetOrCreateStep(db.get(), StepType::ImageLoading, image_cache_key).first,
            feature_cache_key,
            database::GetOrCreateStep(db.get(), StepType::FeatureExtraction, feature_cache_key).first,
            hashing::HashArguments(camera.id.value, camera.config.camera_model, EncodedImages{}),
        });
    }

    testing_utilities::TestDatabaseSetup(context.assets.cameras, camera_test_data, db);

    // Add the intrinsic init manually - this is the only step we are forced to manually add a value to let the rest
    // of the workflow run successfully on the empty data structures.
    for (auto const& camera : context.assets.cameras) {
        auto const [step_id, cache_status]{database::GetOrCreateStep(db.get(), StepType::IntrinsicInit, "")};
        database::IntrinsicInsert(db.get(), step_id, camera.id, camera.config.camera_model,
                                  {Array5d{256, 256, 256, 0, 0.5}});

        // WARN(Jack): This has to match the camera info used inside the TestDatabaseSetup() function. This will mess us
        // up one day! Can we make the connection explicit?
        CameraInfo const camera_info{camera.config.camera_model, {0, 512, 0, 512}};
        database::StepCacheKeyUpdate(db.get(), step_id,
                                     hashing::HashArguments(camera.id.value, camera_info, CameraMeasurements{}));
    }

    ImageInputs const image_inputs{testing_utilities::TestDatabaseImageInputs(context.assets.cameras)};
    // TODO(Jack): Also enable to trigger imu calibration! See warning above.
    EXPECT_NO_THROW(application::Calibrate(config, image_inputs, std::nullopt, db));
}
