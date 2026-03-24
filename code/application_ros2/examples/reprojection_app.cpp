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

    auto const data_path{application::GetCommandOption(argv, argv + argc, "--data")};
    if (not data_path) {
        std::cout << "Missing --data flag" << "\n";
        return EXIT_FAILURE;
    }

    auto const load_config_result{application::LoadAndValidateConfig(*config_path)};
    if (std::holds_alternative<TomlErrorMsg>(load_config_result)) {
        std::cout << std::get<TomlErrorMsg>(load_config_result).msg << "\n";
        return EXIT_FAILURE;
    }


    toml::table const config{ std::get<toml::table>(load_config_result)};

    // NOTE(Jack): Because we validated the config already we do not need to worry about accessing the table directly.
    // TODO(Jack): Should we use the generic templated key access function found in the library?
    std::string const camera_topic{*config["sensor"]["camera_name"].value<std::string>()};
    auto const serialized_topic{ros2::SerializeBagTopic(*data_path, camera_topic)};

    std::cout<< *serialized_topic<<std::endl;



    return 0;
}