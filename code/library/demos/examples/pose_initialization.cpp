
#include <sqlite3.h>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "steps/initialize_calibration.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/calibration_config.hpp"

using namespace reprojection;

struct CameraTestData {
    StepId image_loading_id;
    Hash feature_extraction_key;
    StepId feature_extraction_id;
    Hash camera_info_key;
};

std::array<CameraTestData, 2> const camera_test_data{
    {{
         StepId{1},
         "1d9f6211868fc970b94631f11f02a7110c4008f76a9246dffc86da5098d7b11d",
         StepId{4},
         "a9af3e877da0c5e5d457c51a4302f3e4c2c8891cf7d16a5f5f7c1e547d542e47",
     },
     {
         StepId{2},
         "954f15331b067523ad1792e880ffc349841b1bf4254e18be44f918af3936ea34",
         StepId{5},
         "7c26cad6b72aad7db09fa0b3bf0b09b1db7a1afa8e95de6a5551957b43486540",
     }}};

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.calib.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, false)};

    toml::table const config{toml::parse(testing_utilities::calibration_config)};
    steps::CalibrationContext const context{steps::InitializeCalibration(config, db)};

    // NOTE(Jack): Because we do not have the images themselves checked into the test data, and only the extracted
    // features, we need to "manufacture" cache hits for the image loading, camera info and feature extraction steps.
    // This is essentially what we are doing here in the following block.
    for (std::size_t i{0}; i < context.assets.cameras.size(); ++i) {
        auto const& camera{context.assets.cameras.at(i)};
        auto const& test_data{camera_test_data.at(i)};

        // Write the image loading and feature extraction cache keys
        database::StepCacheKeyUpdate(db.get(), test_data.image_loading_id,
                                     hashing::HashArguments(camera.config.sensor_name));
        database::StepCacheKeyUpdate(db.get(), test_data.feature_extraction_id, test_data.feature_extraction_key);

        // Write the camera info if not already present (allows us to rerun this script without reverting the db).
        auto const [step_id,
                    cache_status]{database::GetOrCreateStep(db.get(), StepType::CameraInfo, test_data.camera_info_key)};
        if (cache_status == CacheStatus::CacheMiss) {
            database::CameraInfoInsert(db.get(), step_id, camera.id,
                                       CameraInfo{
                                           camera.config.camera_model,
                                           {0, 512, 0, 512},
                                       });
            database::StepCacheKeyUpdate(db.get(), step_id, test_data.camera_info_key);
        }
    }

    // Create the two image source inputs.
    application::ImageInputs image_inputs;
    for (auto const& camera : context.assets.cameras) {
        auto const& sensor_name{camera.config.sensor_name};

        // NOTE(Jack): We just use the sensor names as the image source signature but for real application use cases
        // this should be a unique serialized signature dependent on the image data itself. Here we have no images so we
        // can't do that.
        image_inputs.emplace(sensor_name, application::ImageInput{{}, sensor_name});
    }

    application::Calibrate(config, image_inputs, application::ImuInput{{}, ""}, db);

    return EXIT_SUCCESS;
}
