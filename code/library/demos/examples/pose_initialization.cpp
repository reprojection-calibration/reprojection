#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "config/config_parse.hpp"
#include "database/calib_db.hpp"
#include "hashing/hashing.hpp"
#include "steps/initialize_workflow.hpp"
#include "testing_utilities/database_setup_utils.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/calibration_config.hpp"

using namespace reprojection;

// The first entry is for /cam0/image_raw and the second is for /cam1/image_raw
// TODO(Jack): Should we add the image loading cache key here too? Not sure why we calculate this from the sensor name
// separately in the testing utils.
std::vector<testing_utilities::CameraTestData> const camera_test_data{
    {{
         // TODO(Jack): The camera model information here should come from the loaded and parsed config and NOT be
         // hardcoded here!
         CameraInfo{CameraModel::DoubleSphere, {0, 512, 0, 512}},
         "bc0ccd67d52b1ccab91dcf7c06976983af6e14a6f637e19b50636dda1a9ede81",
         StepId{4},
         "0454a740b42831804e23f8a36e6974766b4ce69edbb9f49ed934b87bf5598e97",
     },
     {
         // TODO(Jack): See above! Camera model should come from config!
         CameraInfo{CameraModel::DoubleSphere, {0, 512, 0, 512}},
         "d86c4051b9fb2d711072b40df9b4a5711ec63b394121c186ebae144ba7bf7908",
         StepId{5},
         "a0c7aee3daba97e42a590fcdc5cacb08a9e007f887efd3e416a479e5b471560f",
     }}};

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/dataset-calib-imu4_512_16.calib.db3"};
    auto db{database::OpenCalibDb(record_path, false)};

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
