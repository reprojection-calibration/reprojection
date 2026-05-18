
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "caching/cache_keys.hpp"
#include "calibration/initialization_methods.hpp"
#include "config/config_parsing.hpp"
#include "database/calibration_database.hpp"
#include "database/database_read.hpp"
#include "database/database_write.hpp"
#include "optimization/camera_imu_nonlinear_refinement.hpp"
#include "spline/se3_spline.hpp"
#include "spline/spline_initialization.hpp"

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

    //////////////////
    auto const [sensor_name, camera_model]{config::ParseSensorConfig(*config["sensor"].as_table())};
    Frames const poses{database::ReadPoses(db, CalibrationStep::CameraNonlinearRefinement, sensor_name)};

    // TODO(Jack): Do we also want to save these poses to the db?
    spline::Se3Spline const interpolated_spline{spline::InitializeSe3SplineState(poses)};

    CameraInfo const camera_info{sensor_name, camera_model, {0, 512, 0, 512}};
    auto const camera_measurements{database::ReadExtractedTargets(db, sensor_name)};
    auto const intrinsics{
        database::ReadCameraState(db, CalibrationStep::CameraNonlinearRefinement, sensor_name, camera_model)};

    auto const [frames, errors]{optimization::SplineReprojectionResiduals(
        camera_info, camera_measurements, CameraState{*intrinsics}, interpolated_spline)};

    // TODO(Jack): Is "spline interpolation" the right name?
    database::WriteToDb(CalibrationStep::SplineInterpolation, "", sensor_name, db);
    database::WriteToDb(frames, CalibrationStep::SplineInterpolation, sensor_name, db);
    database::WriteToDb(errors, CalibrationStep::SplineInterpolation, sensor_name, db);

    ImuMeasurements const imu_data{database::ReadImuData(db, "/imu0")};
    auto const [orientation_init, gravity_w]{calibration::EstimateCameraImuRotationAndGravity(
        {interpolated_spline.So3(), interpolated_spline.GetTimeHandler()}, imu_data)};
    auto const [R_imu_co, _]{orientation_init};

    auto const [state, opt_R_imu_co, _1]{optimization::SplineNonlinearRefinement(
        camera_info, camera_measurements, imu_data, CameraState{*intrinsics}, R_imu_co, gravity_w, interpolated_spline)};
    auto const [spline_optimized_intrinsics, optimized_spline]{state};

    std::cout << R_imu_co << std::endl;
    std::cout << opt_R_imu_co << std::endl;

    auto const [frames1, errors1]{optimization::SplineReprojectionResiduals(
        camera_info, camera_measurements, spline_optimized_intrinsics, optimized_spline)};

    database::WriteToDb(CalibrationStep::SplineNonlinearRefinement, "", sensor_name, db);
    database::WriteToDb(frames1, CalibrationStep::SplineNonlinearRefinement, sensor_name, db);
    database::WriteToDb(errors1, CalibrationStep::SplineNonlinearRefinement, sensor_name, db);

    ImuMeasurements spline_imu_data;
    for (auto const& timestamp_ns : imu_data | std::views::keys) {
        auto const position{optimized_spline.Evaluate(timestamp_ns, spline::DerivativeOrder::Null)};
        auto const velocity{optimized_spline.Evaluate(timestamp_ns, spline::DerivativeOrder::First)};
        auto const acceleration{optimized_spline.Evaluate(timestamp_ns, spline::DerivativeOrder::Second)};

        if (position and velocity and acceleration) {
            auto const R_co_w{geometry::Exp<double>(position->topRows(3))};

            // TODO(Jack): Figure out scaling!
            // TODO(Jack): Account for IMU-cam translation! Currently assumes same position!
            ImuData const data_i{R_imu_co * 1e9 * velocity->topRows(3),
                                 R_imu_co * R_co_w * (gravity_w - 1e18 * acceleration->bottomRows(3))};
            spline_imu_data[timestamp_ns] = data_i;
        }
    }

    database::WriteToDb(spline_imu_data, "spline_imu_data", db);

    return EXIT_SUCCESS;
}
