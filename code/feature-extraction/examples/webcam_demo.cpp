#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <iostream>

#include "feature_extraction/target_extraction.hpp"

// To get this working from CLion dev env I followed this link:
// https://medium.com/@steffen.stautmeister/how-to-build-and-run-opencv-and-pytorch-c-with-cuda-support-in-docker-in-clion-6f485155deb8
// After doing that my toolchain "Container Settings" were:
//      -e DISPLAY=:0.0 --entrypoint= -v /tmp/.X11-unix:/tmp/.X11-unix -v /dev:/dev --privileged --rm

using namespace reprojection_calibration::feature_extraction;

// Adopted from https://stackoverflow.com/questions/865668/parsing-command-line-arguments-in-c
char* GetCommandOption(char** begin, char** end, const std::string& option) {
    char** itr = std::find(begin, end, option);
    if (itr != end && ++itr != end) {
        return *itr;
    }

    return 0;
}

int main(int argc, char* argv[]) {
    char const* const filename{GetCommandOption(argv, argv + argc, "-c")};
    if (not filename) {
        std::cerr << "Target configuration yaml not provided! (-c <target_config_yaml>)" << std::endl;
        return EXIT_FAILURE;
    }

    YAML::Node const config{YAML::LoadFile(filename)};
    std::unique_ptr<TargetExtractor> const extractor{CreateTargetExtractor(config["target"])};

    cv::VideoCapture cap(0);
    if (not cap.isOpened()) {
        std::cerr << "Couldn't open video capture device!" << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "\n\tPress any key to close the window and end the demo.\n" << std::endl;

    cv::Mat frame, gray;
    while (true) {
        cap >> frame;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::optional<FeatureFrame> const target{extractor->Extract(gray)};
        if (target.has_value()) {
            Eigen::MatrixX2d const& pixels{target.value().pixels};
            Eigen::ArrayX2i const& indices{target.value().indices};
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