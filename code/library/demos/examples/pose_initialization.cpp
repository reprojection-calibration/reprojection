
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "steps/initialize_calibration.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.calib.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, false)};

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

        // TODO(Jack): Should we also write the image loading and feature extraction keys here? Or should they be
        // hardcoded into the db?

        Hash const camera_info_cache_key{"1cc994b3b1dfe158bed402f46b5de3c00e14bf2d8057f43dd3531eebea5390c5"};
        auto const step_result{database::GetOrCreateStep(db.get(), StepType::CameraInfo, camera_info_cache_key)};

        // NOTE(Jack): We only need to insert the camera info on the first pass when it's a cache miss. If we do it
        // again on subsequent runs we will violate the unique constraint.
        if (step_result.second == CacheStatus::CacheMiss) {
            database::CameraInfoInsert(db.get(), step_result.first, context.camera_id,
                                       CameraInfo{context.config.camera.camera_model, {0, 512, 0, 512}});
            database::StepCacheKeyUpdate(db.get(), step_result.first, camera_info_cache_key);
        }
    } catch (...) {
        std::cerr << "\nDatabase setup threw exception.\n" << std::endl;
    }

    // NOTE(Jack): Because the step type and cache key need to be a unique pair for every step we cannot just use an
    // empty string here because then we could not run this for both cam0 and cam1. If you want to run cam1 you need to
    // update this and possibly write the cache key into the respective step.
    std::string const image_signature{"cam0_image_signature"};

    application::Calibrate(config, {{}, image_signature}, application::ImuInput{{}, ""}, db);

    return EXIT_SUCCESS;
}
