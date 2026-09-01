#include <iostream>

#include <application/reprojection_calibration.hpp>

#include "application_ros2/reprojection.hpp"

using namespace reprojection;

// TODO(Jack): What is a long term strategy to guarantee that these error messages stay consistent across all
//  applications?

int main(int argc, char* argv[]) {
    auto const app_args{application::ParseArgs(argc, argv)};
    if (not app_args) {
        return EXIT_FAILURE;
    }
    application::Sensors const sensors{application::ParseSensors(app_args->config)};

    // NOTE(Jack): We want to control the terminal output of our program entirely. But ROS loves to log so we need to
    // manually set the log level to the highest possible level in an effort to prevent ROS logging for normal errors
    // like not being able to open a bag file. If I knew how to turn off the logging completely I would!
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);

    // NOTE(Jack): ImageSource stores a mutable reference to its SingleTopicBagReader, so the readers need to remain
    // alive and at stable addresses until Calibrate() returns. Reserving the complete camera count prevents the vector
    // from reallocating while we construct the image inputs below.
    std::vector<ros2::SingleTopicBagReader> image_bag_readers;
    image_bag_readers.reserve(std::size(sensors.camera_names));

    ImageInputs image_inputs;
    for (auto const& camera_name : sensors.camera_names) {
        auto image_reader_result{ros2::SingleTopicBagReader::Create(app_args->data_path, camera_name)};
        if (std::holds_alternative<ros2::BagError>(image_reader_result)) {
            std::cerr << std::get<ros2::BagError>(image_reader_result).message << "\n";
            return EXIT_FAILURE;
        }

        image_bag_readers.push_back(std::move(std::get<ros2::SingleTopicBagReader>(image_reader_result)));
        auto& image_bag_reader{image_bag_readers.back()};

        auto const image_signature{ros2::SerializeBagTopic(image_bag_reader)};
        if (not image_signature) {
            std::cerr << "Unable to calculate image data signature for " << camera_name << "!\n";
            return EXIT_FAILURE;
        }

        image_inputs.emplace(camera_name,
                             application::ImageInput{ros2::ImageSource{image_bag_reader}, *image_signature});
    }

    // Early execution and return for the camera only intrinsic only case.
    if (not sensors.imu_name.has_value()) {
        application::Calibrate(app_args->config, image_inputs, std::nullopt, app_args->db);

        return EXIT_SUCCESS;
    }

    auto imu_reader_result{ros2::SingleTopicBagReader::Create(app_args->data_path, *sensors.imu_name)};
    if (std::holds_alternative<ros2::BagError>(imu_reader_result)) {
        std::cerr << std::get<ros2::BagError>(imu_reader_result).message << "\n";
        return EXIT_FAILURE;
    }
    auto& imu_bag_reader{std::get<ros2::SingleTopicBagReader>(imu_reader_result)};

    ros2::ImuSource imu_source{imu_bag_reader};

    auto const imu_signature{ros2::SerializeBagTopic(imu_bag_reader)};
    if (not imu_signature) {
        std::cerr << "Unable to calculate image data signature!\n";
        return EXIT_FAILURE;
    }

    application::Calibrate(app_args->config, image_inputs, application::ImuInput{imu_source, *imu_signature},
                           app_args->db);

    return EXIT_SUCCESS;
}