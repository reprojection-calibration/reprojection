
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

            [target]
            pattern_size = [6,6]
            type = "aprilgrid3"
            unit_dimension = 0.1
        )"};
    toml::table const config{toml::parse(config_file)};

    // TODO(Jack): Move back into try catch block?
    auto const [sensor_name, camera_model]{config::ParseSensorConfig(*config["camera"].as_table())};

    // NOTE(Jack): Because we do not have the images themselves checked into the test data, and only the extracted
    // features, we need to "manufacture" cache hits for the camera info and feature extraction steps. This is
    // essentially what we are doing here in the following block. The reason that we put it into a try catch block is to
    // prevent the database throwing and killing the program when we run the program more than once without resetting
    // the database.
    // TODO(Jack): Is there anyway to avoid hardcoding the cache keys? This is extremely brittle as it stands.

    CameraInfo const camera_info{sensor_name, camera_model, {0, 512, 0, 512}};
    try {
        database::InsertStep(db, sensor_name, CalibrationStep::ImageLoading,
                             "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");

        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::CameraInfo,
                             "1cfeafb06f588d676b115f0ffdb0f601bdfef2e3e604b5ac331a97363e9a993e");
        database::InsertCameraInfo(db, camera_info);

        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::FeatureExtraction,
                             "5d87595c7c8f53d8c355f8b889374c6d1d1cd4bed1472da698725bd51777385a");
    } catch (...) {
        std::cerr << "Database setup threw exception." << std::endl;
    }

    ImageSourceSignature empty_image_source{[]() { return std::nullopt; }};
    application::Calibrate(config, empty_image_source, "", db);

    /////// Hack Workspace Below ////
    /////// Hack Workspace Below ////
    /////// Hack Workspace Below ////
    /////// Hack Workspace Below ////
    CameraMeasurements const targets{database::ReadTargets(db, camera_info.sensor_name)};
    // UNPROTECTED OPTIONAL ACCESS OF THIS VARIABLE!
    auto const intrinsics{database::ReadIntrinsics(db, camera_info.sensor_name, CalibrationStep::BundleAdjustment,
                                                   camera_info.camera_model)};
    Frames const poses{database::ReadPoses(db, sensor_name, CalibrationStep::BundleAdjustment)};

    steps::SplineInitialization const spline_init_step{camera_info, targets, {{*intrinsics}, poses}};
    auto const [spline, spline_init_cache_status]{steps::RunStep<spline::Se3Spline>(spline_init_step, db)};
    std::cout << "Spline init cache: " << ToString(spline_init_cache_status) << std::endl;

    std::string const extrinsic_id{"/imu0"};
    std::string const imu_name{"/imu0"};
    ImuMeasurements const imu_data{database::ReadImuData(db, imu_name)};

    // NOTE(Jack): Has to be the imu name here due to ImuError foreign key constraint.
    steps::ExtrinsicInitialization const extrinsic_init_step{extrinsic_id, imu_name, imu_data, spline};
    auto const [extrinsics,
                extrinsic_init_cache_status]{steps::RunStep<std::pair<Array6d, Array3d>>(extrinsic_init_step, db)};
    std::cout << "Extrinsic init cache: " << ToString(extrinsic_init_cache_status) << std::endl;

    return EXIT_SUCCESS;
}
