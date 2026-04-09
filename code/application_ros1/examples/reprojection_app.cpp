#include <cstdlib>
#include <iostream>

// WARN(Jack): We should be able to include this like #include <reprojection/application/io.hpp> but the install paths
// are not working like we want! We need to take a look at this. We want this so we can prevent file name collisions and
// simply make it more clear where this comes from.
#include <application/calibrate.hpp>
#include <application/io.hpp>

#include "application_ros1/bag_wrapper.hpp"
#include "application_ros1/reprojection.hpp"

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

    auto const db{application::Open(paths->workspace_dir, paths->data_path)};
    if (not db) {
        return EXIT_FAILURE;
    }

    toml::table const config_table{*config};
    // TODO(Jack): Should we use the generic templated key access function found in the library?
    std::string const camera_topic{*config_table["sensor"]["camera_name"].value<std::string>()};

    auto const reader_result{ros1::SingleTopicBagReader::Create(paths->data_path, camera_topic)};
    if (std::holds_alternative<ros1::BagError>(reader_result)) {
        // TODO(Jack): Should we use the libraries logging pattern here too?
        std::cerr << std::get<ros1::BagError>(reader_result).message << "\n";
        return EXIT_FAILURE;
    }
    auto const& bag_reader{std::get<ros1::SingleTopicBagReader>(reader_result)};

    ros1::ImageSource image_source{bag_reader};

    auto const data_signature{SerializeBagTopic(bag_reader)};
    if (not data_signature) {
        std::cerr << "Unable to calculate image data signature!\n";
        return EXIT_FAILURE;
    }

    application::Calibrate(config_table, image_source, *data_signature, *db);

    std::cout << "The future is calibrated!\n";

    return 0;
}