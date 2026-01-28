#include <iostream>

#include "demos/image_source.hpp"
#include "feature_extraction/target_extraction.hpp"
#include "types/eigen_types.hpp"

// To get this working from CLion dev env I followed this link:
// https://medium.com/@steffen.stautmeister/how-to-build-and-run-opencv-and-pytorch-c-with-cuda-support-in-docker-in-clion-6f485155deb8
// After doing that my toolchain "Container Settings" were:
//      -e DISPLAY=:0.0 --entrypoint= -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev --privileged --rm

using namespace reprojection;

// Adopted from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
char* GetCommandOption(char** begin, char** end, const std::string& option) {
    char** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end) {
        return *itr;
    }

    return 0;
}

int main(int argc, char* argv[]) {
    char const* const config_file{GetCommandOption(argv, argv + argc, "-c")};
    if (not config_file) {
        std::cerr << "Target configuration TOML not provided! (-c <target_config_toml>)" << std::endl;
        return EXIT_FAILURE;
    }

    // If no folder is provided then default to webcam demo.
    std::unique_ptr<demos::ImageSource> image_feed;
    char const* const folder{GetCommandOption(argv, argv + argc, "-f")};
    if (folder) {
        image_feed = std::make_unique<demos::ImageFolder>(folder);
    } else {
        std::cout << "Folder not provided! (-f <folder_path>)! Defaulting to webcam demo." << std::endl;
        // TODO(Jack): Provide user option to select a different device
        image_feed = std::make_unique<demos::VideoCapture>(0);
    }

    toml::table const config{toml::parse_file(config_file)};
    auto const extractor{feature_extraction::CreateTargetExtractor(*config["target"].as_table())};

    std::cout << "\n\tPress any key to close the window and end the demo.\n" << std::endl;

    cv::Mat frame, gray;
    while (true) {
        frame = image_feed->GetImage();
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::optional<ExtractedTarget> const target{extractor->Extract(gray)};
        if (target.has_value()) {
            MatrixX2d const& pixels{target->bundle.pixels};
            ArrayX2i const& indices{target->indices};
            for (Eigen::Index i{0}; i < pixels.rows(); ++i) {
                cv::circle(frame, cv::Point(pixels.row(i)[0], pixels.row(i)[1]), 1, cv::Scalar(0, 255, 0), 5,
                           cv::LINE_8);

                std::string const text{"(" + std::to_string(indices.row(i)[0]) + ", " +
                                       std::to_string(indices.row(i)[1]) + ")"};
                cv::putText(frame, text, cv::Point(pixels.row(i)[0], pixels.row(i)[1]), cv::FONT_HERSHEY_COMPLEX, 0.4,
                            cv::Scalar(255, 255, 255), 1);
            }
        }

        cv::imshow("Tag Detections", frame);
        if (cv::waitKey(30) >= 0) {
            break;
        }
    }

    return EXIT_SUCCESS;
}