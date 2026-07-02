#include <iostream>

#include "application/cli_utils.hpp"
#include "application/image_source.hpp"
#include "config/config_parse.hpp"
#include "feature_extraction/target_extraction.hpp"
#include "image_viewer/image_viewer.hpp"

// To get this working from CLion dev env I followed this link:
// https://medium.com/@steffen.stautmeister/how-to-build-and-run-opencv-and-pytorch-c-with-cuda-support-in-docker-in-clion-6f485155deb8
// After doing that my toolchain "Container Settings" were:
//
//      -e DISPLAY=:0.0 --entrypoint= -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev --privileged --rm
//
// You might also need to run `xhost +` from a terminal if it still cannot open the window. If that is really a long
// term solution...?

using namespace reprojection;

// NOTE(Jack): The entire purpose of the demo is to show the featue extraction which is why the display is hardcoded
// here and not controlled by a config file parameter.

int main(int argc, char* argv[]) {
    auto const config_file{application::GetCommandOption(argv, argv + argc, "--config")};
    if (not config_file) {
        std::cerr << "Target configuration TOML not provided! (--config <target_config_toml>)" << std::endl;
        return EXIT_FAILURE;
    }

    // TODO(Jack): Print out to the user that we support the opencv cv::VideoCapture API for input.
    // TODO(Jack): Provide user option to select a different device instead of just the default zero device.
    std::unique_ptr<application::ImageSource> image_feed;
    if (auto const data{application::GetCommandOption(argv, argv + argc, "--data")}) {
        image_feed = std::make_unique<application::VideoCapture>(*data);
    } else {
        std::cout << "Defaulting to webcam demo." << std::endl;
        image_feed = std::make_unique<application::VideoCapture>(0);
    }

    toml::table const config_table{toml::parse_file(*config_file)};

    // TODO(Jack): This conversion logic is now at least repeated here and in the target info step exactly the same,
    // this could be good place for a reusable config parsing function instead of copy and paste.
    auto const cfg{config::Config::Target::Parse(*config_table["target"].as_table())};
    TargetInfo const target_info{cfg.target_type, cfg.size[0], cfg.size[1], cfg.unit_dimension, cfg.asymmetric};

    auto const extractor{feature_extraction::CreateTargetExtractor(target_info)};
    while (true) {
        cv::Mat const img{image_feed->GetImage()};

        std::optional<ExtractedTarget> const target{extractor->Extract(img)};
        if (target.has_value()) {
            feature_extraction::DrawTarget(*target, img);
        }

        static image_viewer::ImageViewer viewer(
            std::make_unique<image_viewer::OpenCvGuiInterface>("Target Feature Extraction"),
            std::make_unique<image_viewer::OpenCvKeyboardInput>());
        viewer.Show(img);
        if (viewer.ShouldQuit()) {
            break;
        }
    }

    return EXIT_SUCCESS;
}