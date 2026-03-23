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
        // TODO(Jack): What is a long term strategy to guarantee that these error messages stay consistent across all
        //  applications?
        std::cout << "\n\tMissing --config flag\n" << std::endl;

        return EXIT_FAILURE;
    }

    auto const config{application::LoadAndValidateConfig(*config_path)};
    if (std::holds_alternative<TomlErrorMsg>(config)) {
        std::cout << "\n\t" + std::get<TomlErrorMsg>(config).msg + "\n" << std::endl;

        return EXIT_FAILURE;
    }

    auto const [bag_file, image_topic]{ros1::DummyLoadConfig()};
    ros1::BagWrapper const bag{bag_file, rosbag::bagmode::Read};

    auto const cache_string{TopicCacheString(bag, image_topic)};
    std::cout << cache_string.value() << std::endl;

    return 0;
}