#include <cstdlib>
#include <iostream>

// WARN(Jack): We should be able to include this like #include <reprojection/application/*.hpp> but the install paths
// are not working like we want! We need to take a look at this. We want this so we can prevent file name collisions and
// simply make it more clear where this comes from.
#include <application/reprojection_calibration.hpp>

#include "application_ros1/bag_wrapper.hpp"
#include "application_ros1/reprojection.hpp"

using namespace reprojection;

int main(int argc, char* argv[]) {
    auto const app_args{application::ParseArgs(argc, argv)};
    if (not app_args) {
        return EXIT_FAILURE;
    }

    // TODO(Jack): Should we use the generic templated key access function found in the library?
    std::string const camera_topic{*app_args->config["sensor"]["camera_name"].value<std::string>()};

    auto const reader_result{ros1::SingleTopicBagReader::Create(app_args->data_path, camera_topic)};
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

    application::Calibrate(app_args->config, image_source, *data_signature, app_args->db);

    std::cout << "The future is calibrated!\n";

    return 0;
}