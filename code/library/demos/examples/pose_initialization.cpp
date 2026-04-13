
#include <ranges>

#include <toml++/toml.hpp>

#include "application/calibrate.hpp"
#include "caching/cache_keys.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, false, false)};

    static constexpr std::string_view config_file{R"(
            [sensor]
            camera_name = "/cam0/image_raw"
            camera_model = "double_sphere"

            [target]
            pattern_size = [3,4]
            type = "circle_grid"
        )"};
    toml::table const config{toml::parse(config_file)};

    // NOTE(Jack): Because we do not have the images themselves checked into the test data, and only the extracted
    // features, we need to "manufacture" cache hits for the camera info and feature extraction steps. This is
    // essentially what we are doing here in the following block. The reason that we put it into a try catch block is to
    // prevent the database throwing and killing the program when we run the program more than once without resetting
    // the database.
    try {
        CameraInfo const camera_info{config["sensor"]["camera_name"].as_string()->get(),
                                     ToCameraModel(config["sensor"]["camera_model"].as_string()->get()),
                                     {0, 512, 0, 512}};
        database::WriteToDb(camera_info, db);

        // NOTE(Jack): Normally the cache key for both camera info and feature extraction steps is the sum of both
        // their respective configurations and the image_source_signature. As we have no image source in this example
        // script we can ignore it and simply pass it as a blank string when we call application::Calibrate(). That is
        // the reason why the step cache keys we create here are only created from their respective configuration
        // tables.
        std::ostringstream oss1;
        oss1 << *config["sensor"].as_table();
        database::WriteToDb(CalibrationStep::CameraInfo, caching::CacheKey(oss1.str()), camera_info.sensor_name, db);

        std::ostringstream oss2;
        oss2 << *config["target"].as_table();
        database::WriteToDb(CalibrationStep::FtEx, caching::CacheKey(oss2.str()), camera_info.sensor_name, db);
    } catch (...) {
    }

    application::Calibrate(config, {}, "", db);

    return EXIT_SUCCESS;
}
