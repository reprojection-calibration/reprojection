#include <iostream>

#include <reprojection/application/io.hpp>
#include <toml++/impl/table.hpp>

#include "application_ros2/reprojection.hpp"

using namespace reprojection;

// TODO(Jack): What is a long term strategy to guarantee that these error messages stay consistent across all
//  applications?
// TODO(Jack): Should we use the generic templated toml key access function found in the library?

int main(int argc, char* argv[]) {
    auto const paths{application::ParseCommandLineInput(argc, argv)};
    if (not paths) {
        return EXIT_FAILURE;
    }

    auto const config{application::LoadAndValidateConfig(paths->config_path)};
    if (not config) {
        return EXIT_FAILURE;
    }

    toml::table const config_table{*config};
    std::string const camera_topic{*config_table["sensor"]["camera_name"].value<std::string>()};

    // NOTE(Jack): We want to control the terminal output of our program entirely. But ROS loves to log so we need to
    // manually set the log level to the highest possible level in an effort to prevent ROS logging for normal errors
    // like not being able to open a bag file. If I knew how to turn off the logging completely I would!
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);

    auto const reader_result{ros2::SingleTopicBagReader::Create(paths->data_path, camera_topic)};
    if (std::holds_alternative<ros2::BagError>(reader_result)) {
        std::cerr << std::get<ros2::BagError>(reader_result).message << "\n";
        return EXIT_FAILURE;
    }

    return 0;
}