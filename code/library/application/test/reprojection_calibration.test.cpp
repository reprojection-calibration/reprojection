#include "application/reprojection_calibration.hpp"

#include <gtest/gtest.h>

#include <memory>

#include "caching/cache_keys.hpp"
#include "config/config_parsing.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/minimum_config.hpp"
#include "testing_utilities/temporary_file.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ApplicationReprojectionCalibration, TestParseArgs) {
    auto result{application::ParseArgs(1, nullptr)};
    EXPECT_FALSE(result.has_value());

    TemporaryFile const config_file{".toml", testing_utilities::minimum_config};

    int const argc{5};

    char const arg0[]{"program"};
    char const arg1[]{"--config"};
    // NOTE(Jack): Guys sorry this got so complicated! But we need to pass the config files path to the parser here in
    // as a char const[], just like how command line args are passed. Getting an actual allocated c style char array
    // a fs::path is not trivial and therefore we have to do this messy stuff here. Can this be simplified?
    auto arg2{std::make_unique<char[]>(std::strlen(config_file.Path().c_str()) + 1)};
    std::strcpy(arg2.get(), config_file.Path().c_str());
    char const arg3[]{"--data"};
    // TODO(Jack): We are implicitly relying on the fact that this directory exists because it is the folder where the
    // TemporaryFile gets created by the fs::temp_directory_path() call. This is ts=a little hacky and it might cause us
    // problems if the assumption turns out not to be true.
    char const arg4[]{"/tmp"};
    char const* const argv[]{arg0, arg1, arg2.get(), arg3, arg4};

    result = application::ParseArgs(argc, argv);
    EXPECT_TRUE(result.has_value());
    EXPECT_EQ(result->data_path, "/tmp");  // Heuristic check of one of the values
}

TEST(ApplicationReprojectionCalibration, TestCalibrate) {
    toml::table const config{toml::parse(testing_utilities::minimum_config)};

    auto db{database::OpenCalibrationDatabase(":memory:", true, false)};

    // TODO(Jack): This test is a little sketchy because we are trying to induce cache hits to avoid actually having to
    // calculate anything. As a principle we do not want to use the checked in test database which means this is as much
    // as we can do here. I guess we could also use the MVG test data generator, but that will be for a future
    // contributor :)

    auto const [sensor_name, camera_model]{config::ParseSensorConfig(*config["camera"].as_table())};
    CameraInfo const camera_info{sensor_name, camera_model, {0, 512, 0, 512}};

    database::InsertStep(db, camera_info.sensor_name, CalibrationStep::ImageLoading, caching::CacheKey(""));

    database::InsertStep(db, camera_info.sensor_name, CalibrationStep::CameraInfo,
                         caching::CacheKey(sensor_name, camera_model, {}));
    database::InsertCameraInfo(db, camera_info);

    database::InsertStep(db, camera_info.sensor_name, CalibrationStep::FeatureExtraction, caching::CacheKey(""));

    database::InsertStep(db, camera_info.sensor_name, CalibrationStep::IntrinsicInitialization,
                         caching::CacheKey(camera_info, {}));
    database::InsertIntrinsics(db, camera_info.sensor_name, CalibrationStep::IntrinsicInitialization,
                               camera_info.camera_model, {Array5d::Zero()});

    // NOTE(Jack): We do not need to do anything for the pose_initialization and bundle_adjustment
    // steps to manufacture a cache hit because if their inputs are empty they themselves will just pass through with no
    // problem. This might change in the future but for now it stands.

    EXPECT_NO_THROW(application::Calibrate(config, {}, "", db));
}
