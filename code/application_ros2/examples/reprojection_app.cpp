#include <iostream>

#include <reprojection/application/io.hpp>
#include <toml++/toml.hpp>

#include "reprojection/reprojection.hpp"

using namespace reprojection;

// TODO(Jack): What is a long term strategy to guarantee that these error messages stay consistent across all
//  applications?
// TODO(Jack): Should we use the generic templated toml key access function found in the library?

int main(int argc, char* argv[]) {
    auto const paths_result{application::ParseCommandLineInput(argc, argv)};
    if (std::holds_alternative<application::CliErrorMsg>(paths_result)) {
        std::cout << std::get<application::CliErrorMsg>(paths_result).msg << "\n";
        return EXIT_FAILURE;
    }
    auto const paths{std::get<application::PathConfig>(paths_result)};

    auto const load_config_result{application::LoadAndValidateConfig(paths.config_path)};
    if (std::holds_alternative<TomlErrorMsg>(load_config_result)) {
        std::cout << std::get<TomlErrorMsg>(load_config_result).msg << "\n";
        return EXIT_FAILURE;
    }

    toml::table const config{std::get<toml::table>(load_config_result)};
    std::string const camera_topic{*config["sensor"]["camera_name"].value<std::string>()};

    // NOTE(Jack): We want to control the terminal output of our program entirely. But ROS loves to log so we need to
    // manually set the log level to the highest possible level in an effort to prevent ROS logging for normal errors
    // like not being able to open a bag file. If I knew how to turn off the logging completely I would!
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);

    auto const reader_result{ros2::SingleTopicBagReader::Create(paths.data_path, camera_topic)};
    if (std::holds_alternative<ros2::BagError>(reader_result)) {
        std::cerr << std::get<ros2::BagError>(reader_result).message << "\n";
        return EXIT_FAILURE;
    }

    return 0;
}