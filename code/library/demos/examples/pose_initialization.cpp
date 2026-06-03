
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "caching/cache_keys.hpp"
#include "config/config_parsing.hpp"
#include "database/calibration_database.hpp"
#include "database/database_read.hpp"  // REMOVE
#include "database/database_write.hpp"
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

    try {
        database::WriteToDb(CalibrationStep::ImageLoading,
                            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", sensor_name, db);

        CameraInfo const camera_info{sensor_name, camera_model, {0, 512, 0, 512}};
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

    ImageSourceSignature empty_image_source{[]() { return std::nullopt; }};
    application::Calibrate(config, empty_image_source, "", db);

    ////
    Frames const poses{database::ReadPoses(db, CalibrationStep::BundleAdjustment, sensor_name)};

    steps::SplineInitialization const spline_init_step{sensor_name, poses, CalibrationStep::SplineInterpolation};
    auto const [spline, spline_init_cache_status]{steps::RunStep<spline::Se3Spline>(spline_init_step, db)};
    std::cout << "Spline init cache: " << ToString(spline_init_cache_status) << std::endl;

    ImuMeasurements const imu_data{database::ReadImuData(db, "/imu0")};

    steps::ExtrinsicInitialization const extrinsic_init_step{
        "tf_co_imu", imu_data, {spline.So3(), spline.GetTimeHandler()}};
    auto const [extrinsics,
                extrinsic_init_cache_status]{steps::RunStep<std::pair<Array6d, Array3d>>(extrinsic_init_step, db)};
    std::cout << "Extrinsic init cache: " << ToString(extrinsic_init_cache_status) << std::endl;

    auto const [tf_imu_co, gravity_w]{extrinsics};
    std::cout<< tf_imu_co.transpose() << std::endl;
    std::cout<< gravity_w.transpose() << std::endl;

    return EXIT_SUCCESS;
}
