#pragma once

#include "config/config_parse.hpp"
#include "types/database_types.hpp"
#include "types/io.hpp"

namespace reprojection::testing_utilities {

// This is approximately the information that you will need to fill out the checked in test database with in order to
// trigger a cache hit so you can run the calibration without the images.
struct CameraTestData {
    StepId image_loading_id;
    Hash feature_extraction_key;
    StepId feature_extraction_id;
    Hash camera_info_key;
};

void TestDatabaseSetup(std::vector<Asset<config::Config::Camera>> const& cameras,
                       std::vector<CameraTestData> const& camera_test_data, SqlitePtr const db);

ImageInputs TestDatabaseImageInputs(std::vector<Asset<config::Config::Camera>> const& cameras);

}  // namespace reprojection::testing_utilities