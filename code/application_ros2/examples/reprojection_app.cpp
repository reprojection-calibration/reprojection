#include <iostream>

#include <application/reprojection_calibration.hpp>
#include <toml++/impl/table.hpp>

#include "application_ros2/reprojection.hpp"

using namespace reprojection;

// TODO(Jack): What is a long term strategy to guarantee that these error messages stay consistent across all
//  applications?
// TODO(Jack): Should we use the generic templated toml key access function found in the library?

int main(int argc, char* argv[]) {
    auto const app_args{application::ParseArgs(argc, argv)};
    if (not app_args) {
        return EXIT_FAILURE;
    }

    std::string const camera_topic{*app_args->config["sensor"]["camera_name"].value<std::string>()};

    // NOTE(Jack): We want to control the terminal output of our program entirely. But ROS loves to log so we need to
    // manually set the log level to the highest possible level in an effort to prevent ROS logging for normal errors
    // like not being able to open a bag file. If I knew how to turn off the logging completely I would!
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);

    // TODO(Jack): As noted in the testing, the const semantics of the ROS2 image source is not really nice, because the
    //  ImageSource needs to take and store a mutable reference. This is not really a huge problem but alone the fact
    //  that we have a non owning mutable reference inside of the ImageSource is a sign we are not doing something
    //  right.
    auto reader_result{ros2::SingleTopicBagReader::Create(app_args->data_path, camera_topic)};
    if (std::holds_alternative<ros2::BagError>(reader_result)) {
        std::cerr << std::get<ros2::BagError>(reader_result).message << "\n";
        return EXIT_FAILURE;
    }
    auto& bag_reader{std::get<ros2::SingleTopicBagReader>(reader_result)};

    ros2::ImageSource image_source{bag_reader};

    auto const data_signature{ros2::SerializeBagTopic(bag_reader)};
    if (not data_signature) {
        std::cerr << "Unable to calculate image data signature!\n";
        return EXIT_FAILURE;
    }

    application::Calibrate(app_args->config, image_source, *data_signature, app_args->db);

    std::cout << "The future is calibrated!\n";

    return 0;
}