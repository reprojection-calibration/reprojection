#include "application/reprojection_calibration.hpp"

#include <gtest/gtest.h>

#include <memory>
#include <ranges>

#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "hashing/hashing.hpp"
#include "steps/initialize_workflow.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"
#include "testing_utilities/database_setup_utils.hpp"
// cppcheck-suppress missingInclude
#include "testing_utilities/generated/calibration_config.hpp"
#include "testing_utilities/temporary_file.hpp"

using namespace reprojection;
using TemporaryFile = testing_utilities::TemporaryFile;

TEST(ApplicationReprojectionCalibration, TestParseArgs) {
    auto result{application::ParseArgs(1, nullptr)};
    EXPECT_FALSE(result.has_value());

    TemporaryFile const config_file{".toml", testing_utilities::calibration_config};

    char const arg0[]{"program"};
    char const arg1[]{"--config"};
    std::string const arg2{config_file.Path().string()};
    char const arg3[]{"--data"};
    // TODO(Jack): We are implicitly relying on the fact that this directory exists because it is the folder where the
    // TemporaryFile gets created by the fs::temp_directory_path() call. This is ts=a little hacky and it might cause us
    // problems if the assumption turns out not to be true.
    char const arg4[]{"/tmp"};
    char const arg5[]{"--workspace"};
    char const arg6[]{"/tmp"};
    char const* const argv[]{arg0, arg1, arg2.c_str(), arg3, arg4, arg5, arg6};

    int const argc{7};
    result = application::ParseArgs(argc, argv);

    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(result->data_path, "/tmp");  // Heuristic check of one of the values
}

TEST(ApplicationReprojectionCalibration, TestParseSensors) {
    toml::table const config{toml::parse(testing_utilities::calibration_config)};

    application::Sensors const sensors{application::ParseSensors(config)};

    EXPECT_EQ(std::size(sensors.camera_names), 2);
    for (size_t i{0}; i < std::size(sensors.camera_names); ++i) {
        auto const& cam_i{sensors.camera_names[i]};
        EXPECT_EQ(cam_i, std::format("/cam{}/image_raw", i));
    }

    ASSERT_TRUE(sensors.imu_name.has_value());
    EXPECT_EQ(*sensors.imu_name, "/imu0");
}

// TODO(Jack): Copy and pasted from the steps test fixture in large part! We should move this to a common testing
// utility and use it in both places.
std::tuple<StepId, StepId, ImageSamples> InsertExtractedTargets(AssetId const camera_id, CameraInfo const& camera_info,
                                                                Intrinsic const& intrinsic, SqlitePtr db) {
    // WARN(Jack): If we add the imu data generation here make sure to use a common duration across both data generation
    // calls!
    auto const [targets, _]{testing_mocks::GenerateMvgData(camera_info, intrinsic, 11, 2)};

    // Initialize empty image data using the target timestamps and then write them to the db to satisfy the foreign
    // key constraint.
    ImageSamples const images{[&targets] {
        ImageSamples images;
        for (auto const timestamp_ns : targets | std::views::keys) {
            images.emplace(timestamp_ns, ImageBuffer{});
        }
        return images;
    }()};
    auto const image_loading_id{database::GetOrCreateStep(db.get(), StepType::ImageLoading, "").first};
    database::ImagesInsert(db.get(), image_loading_id, camera_id, images);

    StepId const target_step_id{database::GetOrCreateStep(db.get(), StepType::FeatureExtraction, "").first};
    database::TargetsInsert(db.get(), target_step_id, image_loading_id, camera_id, targets);

    return {image_loading_id, target_step_id, images};
}

// TODO(Jack): There is a lot of logic here copied from the steps test fixture! The basic idea of what we are trying to
// do is setup test data for two cameras (one pinhole and one double sphere). We should be able to use the same
// implementation for both the steps and here!
TEST(ApplicationReprojectionCalibration, TestCalibrate) {
    toml::table config{toml::parse(testing_utilities::calibration_config)};
    config["cam1"].as_table()->insert_or_assign("camera_model", "pinhole");

    auto db{database::OpenCalibrationDatabase(":memory:", true)};
    steps::CalibrationContext const context{steps::InitializeCalibration(config, db)};

    std::vector<testing_utilities::CameraTestData> camera_test_data;
    for (size_t i{0}; i < std::size(context.assets.cameras); ++i) {
        auto const& camera{context.assets.cameras.at(i)};
        CameraInfo const camera_info{camera.config.camera_model, testing_utilities::image_bounds};

        // TODO(Jack): Copy and pasted hardcoded/hacked logic!
        Intrinsic intrinsic;
        if (camera_info.camera_model == CameraModel::DoubleSphere) {
            intrinsic = {testing_utilities::double_sphere_intrinsics};
        } else if (camera_info.camera_model == CameraModel::Pinhole) {
            intrinsic = {testing_utilities::pinhole_intrinsics};
        } else {
            throw std::runtime_error{std::format("Camera model {} not found!", ToString(camera_info.camera_model))};
        }

        auto const [images_id, targets_id,
                    image_samples]{InsertExtractedTargets(camera.id, camera_info, intrinsic, db)};

        // WARN(Jack): If the camera info cache key calculation method changes then we will need to update this here
        // too (specifically the call to HashArguments())!
        camera_test_data.push_back({
            camera_info,
            images_id,
            hashing::HashArguments(camera.id.value, false, context.assets.target.config, image_samples),
            targets_id,
            hashing::HashArguments(camera.id.value, camera.config.camera_model, image_samples),
        });
    }

    testing_utilities::TestDatabaseSetup(context.assets.cameras, camera_test_data, db);

    ImageInputs const image_inputs{testing_utilities::TestDatabaseImageInputs(context.assets.cameras)};

    // TODO(Jack): Also enable to trigger imu calibration!
    EXPECT_NO_THROW(application::Calibrate(config, image_inputs, std::nullopt, db));
}
