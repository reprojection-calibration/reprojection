#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "config/config_parse.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "testing_mocks/data_generators.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/a1_testing_mocks.db3"};
    auto db{database::OpenCalibrationDatabase(record_path, true, false)};

    static constexpr std::string_view config_file{R"(
            [camera]
            sensor_name = "cam"
            camera_model = "pinhole"

            [imu]
            sensor_name = "imu"

            [target]
            pattern_size = [5, 5]
            type = "checkerboard"
            unit_dimension = 0.1
        )"};
    toml::table const config{toml::parse(config_file)};

    // Because we do not really load any data from the image or imu sources we just hardcode any empty serialization
    // signature.
    std::string const empty{""};

    try {
        auto const cam_cfg{config::Config::Camera::Parse(*config["camera"].as_table())};

        double const duration_s{60};
        CameraInfo const camera_info{cam_cfg.sensor_name, cam_cfg.camera_model, testing_utilities::image_bounds};
        CameraState const intrinsics{testing_utilities::pinhole_intrinsics};
        auto const [targets, camera_frames]{testing_mocks::GenerateMvgData(camera_info, intrinsics, duration_s, 10)};

        // Camera stuff
        database::InsertEntity(db, camera_info.sensor_name, Entity::Camera);

        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::ImageLoading, hashing::Sha256(empty));
        // Insert empty images to satisfy foreign key constraint.
        EncodedImages const images{[&targets]() {
            EncodedImages images;
            for (auto const& timestamp_ns : targets | std::views::keys) {
                images.insert({timestamp_ns, {}});
            }
            return images;
        }()};
        database::InsertImages(db, camera_info.sensor_name, images);

        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::CameraInfo,
                             hashing::HashArguments(camera_info.sensor_name, camera_info.camera_model, images));
        database::InsertCameraInfo(db, camera_info);

        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::FeatureExtraction,
                             "8f3705f71e74c1ed847f7126050b1a16c5178d3a9c0024870c9b7c084d8db0ff");
        database::InsertTargets(db, camera_info.sensor_name, targets);

        // Imu stuff
        if (auto const imu_cfg{config::Config::Imu::Parse(*config["imu"].as_table())}) {
            database::InsertEntity(db, imu_cfg->sensor_name, Entity::Imu);
            database::InsertStep(db, imu_cfg->sensor_name, CalibrationStep::ImuDataLoading, hashing::Sha256(empty));

            auto const [imu_data, _1]{testing_mocks::GenerateImuData(duration_s, 20)};
            database::InsertImuData(db, imu_cfg->sensor_name, imu_data);
        }

    } catch (...) {
        std::cerr << "\nDatabase setup threw exception.\n" << std::endl;
    }

    application::Calibrate(config, {{}, empty}, application::ImuInput{{}, empty}, db);

    return EXIT_SUCCESS;
}
