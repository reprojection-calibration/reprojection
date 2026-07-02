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

    // TODO(Jack): As noted in the testing, the const semantics of the ROS2 image source is not really nice, because the
    //  ImageSource needs to take and store a mutable reference. This is not really a huge problem but alone the fact
    //  that we have a non owning mutable reference inside of the ImageSource is a sign we are not doing something
    //  right.
    auto image_reader_result{ros2::SingleTopicBagReader::Create(app_args->data_path, sensors.camera_sensor)};
    if (std::holds_alternative<ros2::BagError>(image_reader_result)) {
        std::cerr << std::get<ros2::BagError>(image_reader_result).message << "\n";
        return EXIT_FAILURE;
    }
    auto& image_bag_reader{std::get<ros2::SingleTopicBagReader>(image_reader_result)};

    ros2::ImageSource image_source{image_bag_reader};

    auto const image_signature{ros2::SerializeBagTopic(image_bag_reader)};
    if (not image_signature) {
        std::cerr << "Unable to calculate image data signature!\n";
        return EXIT_FAILURE;
    }

    // Early execution and return for the camera only intrinsic only case.
    if (not sensors.imu_sensor.has_value()) {
        application::Calibrate(app_args->config, {image_source, *image_signature}, std::nullopt, app_args->db);

        return EXIT_SUCCESS;
    }

    auto imu_reader_result{ros2::SingleTopicBagReader::Create(app_args->data_path, *sensors.imu_sensor)};
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

    application::Calibrate(app_args->config, {image_source, *image_signature},
                           application::ImuInput{imu_source, *imu_signature}, app_args->db);

    return EXIT_SUCCESS;
}