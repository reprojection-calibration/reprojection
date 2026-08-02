
#include <ranges>

#include <toml++/toml.hpp>

#include "../../steps2/include/steps/initialize_calibration.hpp"
#include "application/reprojection_calibration.hpp"
#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.calib.db3"};
    auto db{database::CalibrationDatabase(record_path, false)};

    static constexpr std::string_view config_file{R"(
            [camera]
            sensor_name = "/cam0/image_raw"
            camera_model = "double_sphere"

            [imu]
            sensor_name = "/imu0"

            [target]
            pattern_size = [6,4]
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
        steps::CalibrationContext const context{steps::InitializeCalibration(config, db)};

        auto step_result{db.GetOrCreateStep(StepType::ImageLoading, Hash{hashing::Sha256("")})};

        step_result = db.GetOrCreateStep(StepType::CameraInfo, "");
        db.CameraInfoInsert(step_result.first, context.camera_id,
                            CameraInfo{context.config.camera.camera_model, {0, 512, 0, 512}});

        step_result = db.GetOrCreateStep(StepType::FeatureExtraction, "");

        // TODO(Jack): Add imu stuff!

    } catch (...) {
        std::cerr << "\nDatabase setup threw exception.\n" << std::endl;
    }

    application::Calibrate(config, {{}, ""}, application::ImuInput{{}, ""}, db);

    return EXIT_SUCCESS;
}
