#include <cstdlib>
#include <iostream>

#include <reprojection/application/cli_utils.hpp>
#include <reprojection/application/load_and_validate_config.hpp>

#include "reprojection/bag_wrapper.hpp"
#include "reprojection/reprojection.hpp"

using namespace reprojection;

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

    auto const reader_result{ros1::SingleTopicBagReader::Create(*bag_path, camera_topic)};
    if (std::holds_alternative<ros1::BagError>(reader_result)) {
        std::cerr << std::get<ros1::BagError>(reader_result).message << "\n";
        return EXIT_FAILURE;
    }

    return 0;
}