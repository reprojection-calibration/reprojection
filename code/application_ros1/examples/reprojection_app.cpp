#include <cstdlib>
#include <iostream>

#include <reprojection/application/cli_utils.hpp>
#include <reprojection/application/database.hpp>
#include <reprojection/application/load_and_validate_config.hpp>

#include "reprojection/bag_wrapper.hpp"
#include "reprojection/reprojection.hpp"

using namespace reprojection;

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
    // TODO(Jack): Should we use the generic templated key access function found in the library?
    std::string const camera_topic{*config["sensor"]["camera_name"].value<std::string>()};

    auto const reader_result{ros1::SingleTopicBagReader::Create(paths.data_path, camera_topic)};
    if (std::holds_alternative<ros1::BagError>(reader_result)) {
        std::cerr << std::get<ros1::BagError>(reader_result).msg << "\n";
        return EXIT_FAILURE;
    }

    auto const db_result{application::Open(paths.workspace_dir, paths.data_path)};
    if (std::holds_alternative<application::DbErrorMsg>(db_result)) {
        std::cerr << std::get<application::DbErrorMsg>(db_result).msg << "\n";
        return EXIT_FAILURE;
    }

    return 0;
}