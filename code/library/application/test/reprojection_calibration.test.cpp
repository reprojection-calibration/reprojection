#include "application/reprojection_calibration.hpp"

#include <gtest/gtest.h>

#include <memory>

#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "steps/initialize_calibration.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/minimum_config.hpp"
#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ApplicationReprojectionCalibration, TestParseArgs) {
    auto result{application::ParseArgs(1, nullptr)};
    EXPECT_FALSE(result.has_value());

    TemporaryFile const config_file{".toml", testing_utilities::minimum_config};

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
    toml::table const config{toml::parse(testing_utilities::minimum_config)};

    application::Sensors const sensors{application::ParseSensors(config)};

    EXPECT_EQ(sensors.camera_sensor, "/cam0/image_raw");
    ASSERT_TRUE(sensors.imu_sensor.has_value());
    EXPECT_EQ(sensors.imu_sensor, "/imu0");
}

TEST(ApplicationReprojectionCalibration, TestCalibrate) {
    toml::table const config{toml::parse(testing_utilities::minimum_config)};
    auto db{database::OpenCalibrationDatabase(":memory:", true)};

    // TODO(Jack): This test is a little sketchy because we are trying to induce cache hits to avoid actually having to
    // calculate anything. As a principle we do not want to use the checked in test database which means this is as much
    // as we can do here. I guess we could also use the MVG test data generator, but that will be for a future
    // contributor :)

    steps::CalibrationContext const context{steps::InitializeCalibration(config, db)};

    std::string const image_sampler_signature{""};
    auto step{database::GetOrCreateStep(db.get(), StepType::ImageLoading, "")};
    database::StepCacheKeyUpdate(db.get(), step.first, {hashing::HashArguments(image_sampler_signature)});

    CameraInfo const camera_info{context.config.camera.camera_model, {0, 512, 0, 512}};
    step = database::GetOrCreateStep(db.get(), StepType::CameraInfo, "");
    database::CameraInfoInsert(db.get(), step.first, context.camera_id, camera_info);
    database::StepCacheKeyUpdate(db.get(), step.first,
                                 hashing::HashArguments(context.config.camera.camera_model, EncodedImages{}));

    step = database::GetOrCreateStep(db.get(), StepType::IntrinsicInit, "");
    database::IntrinsicInsert(db.get(), step.first, context.camera_id, context.config.camera.camera_model,
                              {Array5d{256, 256, 256, 0, 0.5}});
    database::StepCacheKeyUpdate(db.get(), step.first, hashing::HashArguments(camera_info, CameraMeasurements{}));

    // WARN(Jack): I would really really like to also be able to exercise the imu calibration component here but it
    // is not nearly as easy to generate cache hits for those steps with empty inputs/outputs. This requires some
    // more investigation and until then we just need pass std::nullopt for the imu input. NOTE(Jack): We do not
    // need to do anything for the pose_initialization and bundle_adjustment steps to manufacture a cache hit
    // because if their inputs are empty they themselves will just pass through with no problem. This might change
    // in the future but for now it stands.

    // TODO(Jack): Also enable to trigger imu calibration! See warning above.
    EXPECT_NO_THROW(application::Calibrate(config, {{}, image_sampler_signature}, std::nullopt, db));
}
