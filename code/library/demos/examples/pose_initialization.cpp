
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "config/config_parsing.hpp"
#include "database/calibration_database.hpp"
#include "database/database_read.hpp"  // REMOVE
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "steps/extrinsic_initialization.hpp"  // REMOVE
#include "steps/spline_initialization.hpp"     // REMOVE
#include "steps/step_runner.hpp"               // REMOVE

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, false, false)};

    static constexpr std::string_view config_file{R"(
            [camera]
            sensor_name = "/cam0/image_raw"
            camera_model = "double_sphere"

            [imu]
            sensor_name = "/imu0"

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

    try {
        auto const [sensor_name, camera_model]{config::ParseCameraConfig(*config["camera"].as_table())};
        CameraInfo const camera_info{sensor_name, camera_model, {0, 512, 0, 512}};

        // Camera stuff
        database::InsertEntity(db, camera_info.sensor_name, Entity::Camera);

        database::InsertStep(db, sensor_name, CalibrationStep::ImageLoading, hashing::Sha256(""));

        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::CameraInfo,
                             "1cfeafb06f588d676b115f0ffdb0f601bdfef2e3e604b5ac331a97363e9a993e");
        database::InsertCameraInfo(db, camera_info);

        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::FeatureExtraction,
                             "5d87595c7c8f53d8c355f8b889374c6d1d1cd4bed1472da698725bd51777385a");

        // Imu stuff
        auto const imu_name{config::ParseImuConfig(*config["imu"].as_table())};
        database::InsertEntity(db, *imu_name, Entity::Imu);  // Unprotected optional access!!!

        database::InsertStep(db, *imu_name, CalibrationStep::ImuDataLoading, hashing::Sha256(""));
    } catch (...) {
        std::cerr << "\nDatabase setup threw exception.\n" << std::endl;
    }

    ImageSourceSignature empty_image_source{[]() { return std::nullopt; }};
    application::Calibrate(config, empty_image_source, "", db);

    return EXIT_SUCCESS;
}
