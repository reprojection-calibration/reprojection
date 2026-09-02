#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "steps/initialize_workflow.hpp"
#include "testing_utilities/database_setup_utils.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/calibration_config.hpp"

using namespace reprojection;

// The first entry is for /cam0/image_raw and the second is for /cam1/image_raw
// TODO(Jack): Should we add the image loading cache key here too? Not sure why we calculate this from the sensor name
// seperately in the testing utils.
std::vector<testing_utilities::CameraTestData> const camera_test_data{
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
    testing_utilities::TestDatabaseSetup(context.assets.cameras, camera_test_data, db);
    // Create the two empty image source inputs - note that we use the image name as the image signature which
    // conceptually matches our hashing of the sensor name for the image loading in TestDatabaseSetup(), because the
    // image signature will get hashed inside the application itself.
    ImageInputs const image_inputs{testing_utilities::TestDatabaseImageInputs(context.assets.cameras)};

    application::Calibrate(config, image_inputs, ImuInput{{}, ""}, db);

    return EXIT_SUCCESS;
}
