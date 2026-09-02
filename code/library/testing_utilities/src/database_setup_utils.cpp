#include "testing_utilities/database_setup_utils.hpp"

#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"

namespace reprojection::testing_utilities {

void TestDatabaseSetup(std::vector<Asset<config::Config::Camera>> const& cameras,
                       std::vector<CameraTestData> const& camera_test_data, SqlitePtr const db) {
    if (std::size(cameras) != std::size(camera_test_data)) {
        // LCOV_EXCL_START
        throw std::runtime_error(
            std::format("The number of cameras ({}) and the number of camera test data configurations ({}) does not "
                        "match! Cannot setup test database.",
                        std::size(cameras), std::size(camera_test_data)));
        // LCOV_EXCL_STOP
    }

    for (std::size_t i{0}; i < std::size(cameras); ++i) {
        auto const& camera{cameras.at(i)};
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
                                       CameraInfo{camera.config.camera_model, {0, 512, 0, 512}});
            database::StepCacheKeyUpdate(db.get(), step_id, test_data.camera_info_key);
        }
    }
}

ImageInputs TestDatabaseImageInputs(std::vector<Asset<config::Config::Camera>> const& cameras) {
    ImageInputs image_inputs;
    for (auto const& camera : cameras) {
        auto const& sensor_name{camera.config.sensor_name};

        // NOTE(Jack): We just use the sensor names as the image source signature but for real application use cases
        // this should be a unique serialized signature dependent on the image data itself. Here we have no images so we
        // can't do that.
        image_inputs.emplace(sensor_name, ImageInput{{}, sensor_name});
    }

    return image_inputs;
}  // LCOV_EXCL_LINE

}  // namespace reprojection::testing_utilities