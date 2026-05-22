
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "caching/cache_keys.hpp"
#include "config/config_parsing.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
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
        camera_info, CameraState{testing_utilities::pinhole_intrinsics}, 200, timespan_ns)};

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

    return EXIT_SUCCESS;
}
