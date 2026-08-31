
#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "steps/initialize_calibration.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/calibration_config.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.calib.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, false)};

    toml::table const config{toml::parse(testing_utilities::calibration_config)};
    steps::CalibrationContext const context{steps::InitializeCalibration(config, db)};

    // NOTE(Jack): Because we do not have the images themselves checked into the test data, and only the extracted
    // features, we need to "manufacture" cache hits for the camera info and feature extraction steps. This is
    // essentially what we are doing here in the following block. The reason that we put it into a try catch block is to
    // prevent the database throwing and killing the program when we run the program more than once without resetting
    // the database.

    // TODO(Jack): Should we also write the image loading and feature extraction keys here? Or should they be
    // hardcoded into the db?
    database::StepCacheKeyUpdate(db.get(), StepId{1},
                                 hashing::HashArguments(context.assets.cameras.at(0).config.sensor_name));
    database::StepCacheKeyUpdate(db.get(), StepId{2},
                                 hashing::HashArguments(context.assets.cameras.at(1).config.sensor_name));
    database::StepCacheKeyUpdate(db.get(), StepId{4},
                                 "1d9f6211868fc970b94631f11f02a7110c4008f76a9246dffc86da5098d7b11d");
    database::StepCacheKeyUpdate(db.get(), StepId{5},
                                 "954f15331b067523ad1792e880ffc349841b1bf4254e18be44f918af3936ea34");

    Hash const camera_info_cache_key1{"a9af3e877da0c5e5d457c51a4302f3e4c2c8891cf7d16a5f5f7c1e547d542e47"};
    auto const step_result1{database::GetOrCreateStep(db.get(), StepType::CameraInfo, camera_info_cache_key1)};

    // NOTE(Jack): We only need to insert the camera info on the first pass when it's a cache miss. If we do it
    // again on subsequent runs we will violate the unique constraint.
    if (step_result1.second == CacheStatus::CacheMiss) {
        // TODO HANDLE BOTH CAMERAS!!!!
        auto const camera_0{context.assets.cameras[0]};
        database::CameraInfoInsert(db.get(), step_result1.first, camera_0.id,
                                   CameraInfo{camera_0.config.camera_model, {0, 512, 0, 512}});
        database::StepCacheKeyUpdate(db.get(), step_result1.first, camera_info_cache_key1);
    }

    Hash const camera_info_cache_key2{"7c26cad6b72aad7db09fa0b3bf0b09b1db7a1afa8e95de6a5551957b43486540"};
    auto const step_result2{database::GetOrCreateStep(db.get(), StepType::CameraInfo, camera_info_cache_key2)};

    if (step_result2.second == CacheStatus::CacheMiss) {
        auto const camera_1{context.assets.cameras[1]};
        database::CameraInfoInsert(db.get(), step_result2.first, camera_1.id,
                                   CameraInfo{camera_1.config.camera_model, {0, 512, 0, 512}});
        database::StepCacheKeyUpdate(db.get(), step_result2.first, camera_info_cache_key2);
    }

    // NOTE(Jack): We just use the sensor names as the image source signature but for real application use cases this
    // should be a unique serialized signature dependent on the image data itself. Here we have no images so we can't do
    // that.
    application::ImageInputs const image_inputs{
        {context.assets.cameras.at(0).config.sensor_name, {{}, context.assets.cameras.at(0).config.sensor_name}},
        {context.assets.cameras.at(1).config.sensor_name, {{}, context.assets.cameras.at(1).config.sensor_name}}};

    application::Calibrate(config, image_inputs, application::ImuInput{{}, ""}, db);

    return EXIT_SUCCESS;
}
