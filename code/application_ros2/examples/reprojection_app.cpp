#include <iostream>

#include <reprojection/application/cli_utils.hpp>
#include <reprojection/application/load_and_validate_config.hpp>

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

    auto const config{application::LoadAndValidateConfig(*config_path)};
    if (std::holds_alternative<TomlErrorMsg>(config)) {
        std::cout << std::get<TomlErrorMsg>(config).msg << "\n";
        return EXIT_FAILURE;
    }

    return 0;
}