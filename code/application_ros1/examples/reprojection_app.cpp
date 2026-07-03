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

    auto const image_data_signature{ros1::SerializeBagTopic(image_bag_reader)};
    if (not image_data_signature) {
        std::cerr << "Unable to calculate image data signature!\n";
        return EXIT_FAILURE;
    }

    // Early execution and return for the camera only intrinsic only case.
    if (not sensors.imu_sensor.has_value()) {
        application::Calibrate(app_args->config, {image_source, *image_data_signature}, std::nullopt, app_args->db);

        return EXIT_SUCCESS;
    }

    auto const imu_reader_result{ros1::SingleTopicBagReader::Create(app_args->data_path, *sensors.imu_sensor)};
    if (std::holds_alternative<ros1::BagError>(imu_reader_result)) {
        std::cerr << std::get<ros1::BagError>(imu_reader_result).message << "\n";
        return EXIT_FAILURE;
    }
    auto const& imu_bag_reader{std::get<ros1::SingleTopicBagReader>(imu_reader_result)};

    ros1::ImuSource imu_source{imu_bag_reader};

    // TODO(Jack): Right now the only way for the ROS1 app that we detect the bag does not contain the topic is here
    // when we try to calculate the signature but find the topic is empty. We should not wait until this point! If the
    // topic is not present then we should not even allow the construction of the SingleTopicBagReader like how we do in
    // the ros2 app.
    auto const imu_data_signature{ros1::SerializeBagTopic(imu_bag_reader)};
    if (not imu_data_signature) {
        std::cerr << "Unable to calculate imu data signature! Are you sure the topic is in the bag?\n";
        return EXIT_FAILURE;
    }

    // Camera-imu extrinsic and intrinsic case.
    application::Calibrate(app_args->config, {image_source, *image_data_signature},
                           application::ImuInput{imu_source, *imu_data_signature}, app_args->db);

    return EXIT_SUCCESS;
}