
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "caching/cache_keys.hpp"
#include "config/config_parsing.hpp"
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
            pattern_size = [6,6]
            type = "aprilgrid3"
            unit_dimension = 0.1
        )"};
    toml::table const config{toml::parse(config_file)};

    // NOTE(Jack): Because we do not have the images themselves checked into the test data, and only the extracted
    // features, we need to "manufacture" cache hits for the camera info and feature extraction steps. This is
    // essentially what we are doing here in the following block. The reason that we put it into a try catch block is to
    // prevent the database throwing and killing the program when we run the program more than once without resetting
    // the database.
    // TODO(Jack): Is there anyway to avoid hardcoding the cache keys? This is extremely brittle as it stands.

    try {
        auto const [camera_name, camera_model]{config::ParseSensorConfig(*config["sensor"].as_table())};

        database::WriteToDb(CalibrationStep::ImageLoading,
                            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", camera_name, db);

        CameraInfo const camera_info{camera_name, camera_model, {0, 512, 0, 512}};
        database::WriteToDb(CalibrationStep::CameraInfo,
                            "1cfeafb06f588d676b115f0ffdb0f601bdfef2e3e604b5ac331a97363e9a993e", camera_info.sensor_name,
                            db);
        database::WriteToDb(camera_info, db);

        database::WriteToDb(CalibrationStep::FeatureExtraction,
                            "5d87595c7c8f53d8c355f8b889374c6d1d1cd4bed1472da698725bd51777385a", camera_info.sensor_name,
                            db);
    } catch (...) {
        std::cerr << "Database setup threw exception." << std::endl;
    }

    ImageSource empty_image_source{[]() { return std::nullopt; }};
    application::Calibrate(config, empty_image_source, "", db);

    return EXIT_SUCCESS;
}
