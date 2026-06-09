
#include <ranges>

#include <toml++/toml.hpp>

#include "application/reprojection_calibration.hpp"
#include "config/config_parsing.hpp"
#include "database/calibration_database.hpp"
#include "database/database_write.hpp"
#include "hashing/hashing.hpp"
#include "testing_mocks/imu_data_generator.hpp"
#include "testing_mocks/mvg_data_generator.hpp"
#include "testing_utilities/constants.hpp"

using namespace reprojection;

int main() {
    // ERROR(Jack): Hardcoded to work in clion, is there a reproducible way to do this, or at least some philosophy we
    // can officially document?
    std::string const record_path{"/tmp/reprojection/code/test_data/testing_mocks.db3"};
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

    std::string const image_hash{""};

    try {
        uint64_t const timespan_ns{10000000000};
        auto const [sensor_name, camera_model]{config::ParseCameraConfig(*config["camera"].as_table())};

        CameraInfo const camera_info{sensor_name, camera_model, testing_utilities::image_bounds};
        CameraState const intrinsics{testing_utilities::pinhole_intrinsics};
        auto const [targets, camera_frames]{testing_mocks::GenerateMvgData(camera_info, intrinsics, 200, timespan_ns)};

        // Camera stuff
        database::InsertEntity(db, camera_info.sensor_name, Entity::Camera);

        database::InsertStep(db, camera_info.sensor_name, CalibrationStep::ImageLoading, hashing::Sha256(""));
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
                             "532eb1a35212026c31475ec9e2c68b6e0c701ac96ac9c40e615f648f3a6d8317");
        database::InsertTargets(db, camera_info.sensor_name, targets);

        // Imu stuff
        auto const imu_name{config::ParseImuConfig(*config["imu"].as_table())};
        database::InsertEntity(db, *imu_name, Entity::Imu);  // Unprotected optional access!!!
        database::InsertStep(db, *imu_name, CalibrationStep::ImuDataLoading, hashing::Sha256(""));

        auto const [imu_data, _1]{testing_mocks::GenerateImuData(1000, timespan_ns)};
        database::InsertImuData(db, *imu_name, imu_data);
    } catch (...) {
        std::cerr << "\nDatabase setup threw exception.\n" << std::endl;
    }

    ImageSourceSignature empty_image_source{[]() { return std::nullopt; }};
    application::Calibrate(config, empty_image_source, image_hash, db);

    return EXIT_SUCCESS;
}
