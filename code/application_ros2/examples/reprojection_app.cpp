#include <iostream>

#include <reprojection/application/cli_utils.hpp>
#include <reprojection/application/load_and_validate_config.hpp>
#include <toml++/impl/table.hpp>

#include "reprojection/reprojection.hpp"

using namespace reprojection;

// TODO(Jack): What is a long term strategy to guarantee that these error messages stay consistent across all
//  applications?

int main(int argc, char* argv[]) {
    auto const config_path{application::GetCommandOption(argv, argv + argc, "--config")};
    if (not config_path) {
        std::cout << "Missing --config flag" << "\n";
        return EXIT_FAILURE;
    }

    auto const bag_path{application::GetCommandOption(argv, argv + argc, "--data")};
    if (not bag_path) {
        std::cout << "Missing --data flag" << "\n";
        return EXIT_FAILURE;
    }

    auto const load_config_result{application::LoadAndValidateConfig(*config_path)};
    if (std::holds_alternative<TomlErrorMsg>(load_config_result)) {
        std::cout << std::get<TomlErrorMsg>(load_config_result).msg << "\n";
        return EXIT_FAILURE;
    }

    toml::table const config{std::get<toml::table>(load_config_result)};
    // TODO(Jack): Should we use the generic templated key access function found in the library?
    std::string const camera_topic{*config["sensor"]["camera_name"].value<std::string>()};

    // NOTE(Jack): We want to control the terminal output of our program entirely. But ROS loves to log so we need to
    // manually set the log level to the highest possible level in an effort to prevent ROS logging for normal errors
    // like not being able to open a bag file. If I knew how to turn off the logging completely I would!
    rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_FATAL);

    auto const reader_result{ros2::SingleTopicBagReader::Create(*bag_path, camera_topic)};
    if (std::holds_alternative<ros2::BagError>(reader_result)) {
        std::cerr << std::get<ros2::BagError>(reader_result).message << "\n";
        return EXIT_FAILURE;
    }

    return 0;
}