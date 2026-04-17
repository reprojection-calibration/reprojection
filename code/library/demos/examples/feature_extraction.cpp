#include <iostream>

#include "application/io.hpp"
#include "demos/image_source.hpp"
#include "feature_extraction/target_extraction.hpp"
#include "types/eigen_types.hpp"

// To get this working from CLion dev env I followed this link:
// https://medium.com/@steffen.stautmeister/how-to-build-and-run-opencv-and-pytorch-c-with-cuda-support-in-docker-in-clion-6f485155deb8
// After doing that my toolchain "Container Settings" were:
//      -e DISPLAY=:0.0 --entrypoint= -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev --privileged --rm

using namespace reprojection;

// NOTE(Jack): The entire purpose of the demo is to show the featue extraction which is why the display is hardcoded
// here and not controlled by a config file parameter.

int main(int argc, char* argv[]) {
    auto const config_file{application::GetCommandOption(argv, argv + argc, "--config")};
    if (not config_file) {
        std::cerr << "Target configuration TOML not provided! (--config <target_config_toml>)" << std::endl;
        return EXIT_FAILURE;
    }

    // If no folder is provided then default to webcam demo.
    std::unique_ptr<demos::ImageSource> image_feed;

    if (auto const folder{application::GetCommandOption(argv, argv + argc, "--folder")}) {
        image_feed = std::make_unique<demos::ImageFolder>(*folder);
    } else if (auto const file{application::GetCommandOption(argv, argv + argc, "--file")}) {
        image_feed = std::make_unique<demos::VideoCapture>(*file);
    } else {
        std::cout << "Folder not provided! (--folder <folder_path>)! Defaulting to webcam demo." << std::endl;
        // TODO(Jack): Provide user option to select a different device
        image_feed = std::make_unique<demos::VideoCapture>(0);
    }

    toml::table const config{toml::parse_file(*config_file)};
    auto const extractor{feature_extraction::CreateTargetExtractor(*config["target"].as_table())};

    std::cout << "\n\tPress any key to close the window and end the demo.\n" << std::endl;

    cv::Mat frame, gray;
    while (true) {
        frame = image_feed->GetImage();
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::optional<ExtractedTarget> const target{extractor->Extract(gray)};
        if (target.has_value()) {
            feature_extraction::DrawTarget(*target, frame);
        }

        cv::imshow("Tag Detections", frame);
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }

    return EXIT_SUCCESS;
}