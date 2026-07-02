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
    application::Sensors const sensors{application::ParseSensors(app_args->config)};

    auto const image_reader_result{ros1::SingleTopicBagReader::Create(app_args->data_path, sensors.camera_sensor)};
    if (std::holds_alternative<ros1::BagError>(image_reader_result)) {
        // TODO(Jack): Should we use the libraries logging pattern here too?
        std::cerr << std::get<ros1::BagError>(image_reader_result).message << "\n";
        return EXIT_FAILURE;
    }
    auto const& image_bag_reader{std::get<ros1::SingleTopicBagReader>(image_reader_result)};

    ros1::ImageSource image_source{image_bag_reader};

    auto const data_signature{ros1::SerializeBagTopic(image_bag_reader)};
    if (not data_signature) {
        std::cerr << "Unable to calculate image data signature!\n";
        return EXIT_FAILURE;
    }

    // TODO HACKY HACKS FOR LIFE TIME PRESERVATION OF THE BAG READER
    std::unique_ptr<std::variant<ros1::SingleTopicBagReader, ros1::BagError>> imu_reader_result;
    std::optional<application::ImuInput> imu_input{std::nullopt};
    if (sensors.imu_sensor.has_value()) {
        imu_reader_result = std::make_unique<std::variant<ros1::SingleTopicBagReader, ros1::BagError>>(
            ros1::SingleTopicBagReader::Create(app_args->data_path, *sensors.imu_sensor));
        if (std::holds_alternative<ros1::BagError>(*imu_reader_result)) {
            std::cerr << std::get<ros1::BagError>(*imu_reader_result).message << "\n";
            return EXIT_FAILURE;
        }
        auto const& imu_bag_reader{std::get<ros1::SingleTopicBagReader>(*imu_reader_result)};

        ros1::ImuSource imu_source{imu_bag_reader};

        auto const imu_signature{ros1::SerializeBagTopic(imu_bag_reader)};
        if (not imu_signature) {
            std::cerr << "Unable to calculate image data signature!\n";
            return EXIT_FAILURE;
        }

        imu_input = application::ImuInput{imu_source, *imu_signature};
    }

    // TODO(Jack): Should we put this in a try catch block and return EXIT_FAILURE?
    application::Calibrate(app_args->config, {image_source, *data_signature}, imu_input, app_args->db);

    return EXIT_SUCCESS;
}