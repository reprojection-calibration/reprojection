
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "caching/cache_keys.hpp"
#include "config/config_parsing.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "spline/se3_spline.hpp"
#include "spline/spline_evaluation.hpp"
#include "spline/spline_initialization.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/a1_test_data.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, true, false)};

    CameraInfo const camera_info{"cam1", CameraModel::Pinhole, testing_utilities::image_bounds};
    uint64_t const timespan_ns{10000000000};
    auto const [targets, camera_frames]{testing_mocks::GenerateMvgData(
        camera_info, CameraState{testing_utilities::pinhole_intrinsics}, 200 , timespan_ns)};

    EncodedImages image_data;
    for (auto const timestamp_ns : targets | std::views::keys) {
        image_data[timestamp_ns] = {};
    }

    database::WriteToDb(CalibrationStep::ImageLoading, "", camera_info.sensor_name, db);
    database::WriteToDb(image_data, camera_info.sensor_name, db);

    database::WriteToDb(CalibrationStep::CameraInfo, "", camera_info.sensor_name, db);
    database::WriteToDb(camera_info, db);

    // WARN(Jack): The test data target has points at negative coordinates but setting negative bounds in the dashboard
    // is not possible so the target visualization is cut off.
    database::WriteToDb(CalibrationStep::TargetInfo, "", camera_info.sensor_name, db);
    TargetInfo const target_info{TargetType::Checkerboard, 5, 5, 0.25, false};
    database::WriteToDb(target_info, camera_info.sensor_name, db);

    database::WriteToDb(CalibrationStep::FeatureExtraction, "", camera_info.sensor_name, db);
    database::WriteToDb(targets, camera_info.sensor_name, db);

    database::WriteToDb(CalibrationStep::LinearPoseInitialization, "", camera_info.sensor_name, db);
    database::WriteToDb(camera_frames, CalibrationStep::LinearPoseInitialization, camera_info.sensor_name, db);

    uint64_t const num_imu_data{1000};
    ImuMeasurements const imu_data{testing_mocks::GenerateImuData(num_imu_data, timespan_ns)};

    std::string const imu_name{"imu1"};
    database::WriteToDb(imu_data, imu_name, db);

    spline::Se3Spline const interpolated_spline{spline::InitializeSe3SplineState(camera_frames)};

    // WHAT ABOUT THE INVERSE IN THE CAMERA MVG DATA GEN!???

    ImuMeasurements interpolated_frames;  // HACK TO STORE POSE DATA IN IMU DATA!
    ImuMeasurements interpolated_imu_data;
    for (uint64_t const timestamp_ns : imu_data | std::views::keys) {
        auto const tf_w_co{interpolated_spline.Evaluate(timestamp_ns, spline::DerivativeOrder::Null)};
        auto const omega_i_co{spline::EvaluateSpline<spline::So3Spline>(interpolated_spline.So3(),
                                                                        interpolated_spline.GetTimeHandler(),
                                                                        timestamp_ns, spline::DerivativeOrder::First)};
        auto const a_i_co{spline::EvaluateSpline<spline::R3Spline>(interpolated_spline.R3(),
                                                                   interpolated_spline.GetTimeHandler(), timestamp_ns,
                                                                   spline::DerivativeOrder::Second)};

        if (tf_w_co.has_value() and omega_i_co.has_value() and a_i_co.has_value()) {
            auto const R_w_co{geometry::Exp(tf_w_co->topRows(3))};

            interpolated_frames[timestamp_ns] = ImuData{tf_w_co->topRows(3), tf_w_co->bottomRows(3)};
            interpolated_imu_data[timestamp_ns] = ImuData{R_w_co * omega_i_co.value(), R_w_co * a_i_co.value()};
        }
    }

    database::WriteToDb(interpolated_frames, "interpolated_frames", db);
    database::WriteToDb(interpolated_imu_data, "interpolated", db);

    return EXIT_SUCCESS;
}
