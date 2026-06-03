
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

void WriteMvgData(SqlitePtr db, uint64_t const timespan_ns);

void WriteImuData(SqlitePtr db, uint64_t const timespan_ns);

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/test_data.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, true, false)};

    uint64_t const timespan_ns{10000000000};

    WriteMvgData(db, timespan_ns);
    WriteImuData(db, timespan_ns);

    return EXIT_SUCCESS;
}

void WriteMvgData(SqlitePtr db, uint64_t const timespan_ns) {
    CameraInfo const camera_info{"cam1", CameraModel::Pinhole, testing_utilities::image_bounds};
    auto const [targets, camera_frames]{testing_mocks::GenerateMvgData(
        camera_info, CameraState{testing_utilities::pinhole_intrinsics}, 200, timespan_ns)};

    // We need to satisfy the foreign key requirements here and below.
    EncodedImages image_data;
    for (auto const timestamp_ns : targets | std::views::keys) {
        image_data[timestamp_ns] = {};
    }

    database::InsertStep(CalibrationStep::ImageLoading, "", camera_info.sensor_name, db);
    database::InsertImages(image_data, camera_info.sensor_name, db);

    database::InsertStep(CalibrationStep::CameraInfo, "", camera_info.sensor_name, db);
    database::InsertCameraInfo(camera_info, db);

    // WARN(Jack): The test data target has points at negative coordinates but setting negative bounds in the dashboard
    // is not possible so the target visualization is cut off.
    database::InsertStep(CalibrationStep::TargetInfo, "", camera_info.sensor_name, db);
    // TODO(Jack): It would be nice if the mvg data generator used and returned us the target info. Hardcoding it here
    // means that it will go out of sync with the data generator.
    TargetInfo const target_info{TargetType::Checkerboard, 5, 5, 0.25, false};
    database::WriteToDb(target_info, camera_info.sensor_name, db);

    database::InsertStep(CalibrationStep::FeatureExtraction, "", camera_info.sensor_name, db);
    database::InsertTargets(targets, camera_info.sensor_name, db);

    database::InsertStep(CalibrationStep::PoseInitialization, "", camera_info.sensor_name, db);
    database::InsertPoses(camera_frames, CalibrationStep::PoseInitialization, camera_info.sensor_name, db);
}

void WriteImuData(SqlitePtr db, uint64_t const timespan_ns) {
    uint64_t const num_imu_data{1000};
    auto const [imu_data, _]{testing_mocks::GenerateImuData(num_imu_data, timespan_ns)};

    std::string const imu_name{"imu1"};
    database::WriteToDb(imu_data, imu_name, db);
}