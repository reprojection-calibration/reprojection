#include "application/reprojection_calibration.hpp"

#include <gtest/gtest.h>

#include <memory>

#include "caching/cache_keys.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "testing_utilities/temporary_file.hpp"
#include "types/calibration_types.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ApplicationReprojectionCalibration, TestParseArgs) {
    auto result{application::ParseArgs(1, nullptr)};
    EXPECT_FALSE(result.has_value());

    // TODO(Jack): This is now copy and pasted in three places, should we make one common def in the testing utils?
    static constexpr std::string_view minimum_config{R"(
        [sensor]
        camera_name = "/cam0/image_raw"
        camera_model = "double_sphere"

        [target]
        pattern_size = [3,4]
        type = "circle_grid"
    )"};
    TemporaryFile const config_file{".toml", minimum_config};

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
    // TemporaryFile gets created by the fs::temp_directory_path() call. This is a little hacky and it might cause us
    // problems if the assumption turns out not to be true.
    char const arg4[]{"/tmp"};
    char const* const argv[]{arg0, arg1, arg2.get(), arg3, arg4};

    result = application::ParseArgs(argc, argv);
    EXPECT_TRUE(result.has_value());
}

TEST(Application, TestCalibrate) {
    // TODO(Jack): This is now copy and pasted in a lot of places right? Should we do a central definition?
    static constexpr std::string_view config_file{R"(
            [sensor]
            camera_name = "/cam0/image_raw"
            camera_model = "double_sphere"

            [target]
            pattern_size = [3,4]
            type = "circle_grid"
        )"};
    toml::table const config{toml::parse(config_file)};

    auto db{database::OpenCalibrationDatabase(":memory:", true, false)};

    // TODO(Jack): This test is a little sketchy because we are trying to induce cache hits to avoid actually having to
    // calculate anything. As a principle we do not want to use the checked in test database which means this is as much
    // as we can do here. I guess we could also use the MVG test data generator, but that will be for a future
    // contributor :)

    // Write the camera info
    // TODO(Jack): Add foreign key constraint that a camera info must have a step!
    CameraInfo const camera_info{config["sensor"]["camera_name"].as_string()->get(),
                                 ToCameraModel(config["sensor"]["camera_model"].as_string()->get()),
                                 {0, 512, 0, 512}};
    database::WriteToDb(camera_info, db);

    database::WriteToDb(CalibrationStep::ImageLoading, caching::CacheKey(""), camera_info.sensor_name, db);

    std::ostringstream oss1;
    oss1 << *config["sensor"].as_table();
    database::WriteToDb(CalibrationStep::CameraInfo, caching::CacheKey(oss1.str()), camera_info.sensor_name, db);

    std::ostringstream oss2;
    oss2 << *config["target"].as_table();
    database::WriteToDb(CalibrationStep::FtEx, caching::CacheKey(oss2.str()), camera_info.sensor_name, db);

    database::WriteToDb(CalibrationStep::Ii, caching::CacheKey(camera_info, {}), camera_info.sensor_name, db);
    database::WriteToDb({Array6d::Zero()}, camera_info.camera_model, CalibrationStep::Ii, camera_info.sensor_name, db);

    // NOTE(Jack): We do not need to do anything for the linear_pose_initialization and camera_nonlinear_refinement
    // steps to manufacture a cache hit because if their inputs are empty they themselves will just pass through with no
    // problem. This might change in the future but for now it stands.

    EXPECT_NO_THROW(application::Calibrate(config, {}, "", db));
}
