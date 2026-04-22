
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
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
    // TODO(Jack): Is there anyway to avoid hardcoding the cache keys? This is extremely brittle as it stands.

    try {
        std::string const sensor_name{config["sensor"]["camera_name"].as_string()->get()};
        database::WriteToDb(CalibrationStep::ImageLoading,
                            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", sensor_name, db);

        CameraInfo const camera_info{
            sensor_name, ToCameraModel(config["sensor"]["camera_model"].as_string()->get()), {0, 512, 0, 512}};

        std::ostringstream oss1;
        oss1 << *config["sensor"].as_table();
        database::WriteToDb(CalibrationStep::CameraInfo,
                            "f9dfdc874264f36f71b5d06f19787ac477de30e3808a0fbb14280c5fd1b0e647", camera_info.sensor_name,
                            db);
        database::WriteToDb(camera_info, db);

        std::ostringstream oss2;
        oss2 << *config["target"].as_table();
        database::WriteToDb(CalibrationStep::FtEx, "049b921634ea226820738b1b9c0f3cec0afe60868e6309cb01196a2787b65591",
                            camera_info.sensor_name, db);
    } catch (...) {
        std::cerr << "Database setup threw exception." << std::endl;
    }

    ImageSource empty_image_source{[]() { return std::nullopt; }};
    application::Calibrate(config, empty_image_source, "", db);

    return EXIT_SUCCESS;
}
