#include <cstdlib>
#include <iostream>

#include <reprojection/application/io.hpp>

#include "reprojection/bag_wrapper.hpp"
#include "reprojection/reprojection.hpp"

using namespace reprojection;

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
    // TODO(Jack): Should we use the generic templated key access function found in the library?
    std::string const camera_topic{*config_table["sensor"]["camera_name"].value<std::string>()};

    auto const reader_result{ros1::SingleTopicBagReader::Create(paths->data_path, camera_topic)};
    if (std::holds_alternative<ros1::BagError>(reader_result)) {
        std::cerr << std::get<ros1::BagError>(reader_result).message << "\n";
        return EXIT_FAILURE;
    }

    return 0;
}