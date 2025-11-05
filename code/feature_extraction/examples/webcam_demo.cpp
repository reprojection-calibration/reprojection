#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <iostream>

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

class ImageFeed {
   public:
    virtual ~ImageFeed() = default;

    virtual cv::Mat GetImage() = 0;
};

class WebcamFeed : public ImageFeed {
   public:
    WebcamFeed() {
        cap_ = cv::VideoCapture(0);
        if (not cap_.isOpened()) {
            throw std::runtime_error("Couldn't open video capture device!");
        }
    }

    cv::Mat GetImage() override {
        cv::Mat frame;
        cap_ >> frame;
        return frame;
    }

   private:
    cv::VideoCapture cap_;
};

class FolderFeed : public ImageFeed {
   public:
    FolderFeed(std::string const& image_folder) {
        for (const auto& entry : std::filesystem::directory_iterator(image_folder)) {
            image_files_.push_back(entry.path());
        }

        std::sort(std::begin(image_files_), std::end(image_files_));
    }

    cv::Mat GetImage() override {
        if (current_id_ >= std::size(image_files_)) {
            // Out of images to load, return empty.
            return cv::Mat();
        }

        cv::Mat const image{cv::imread(image_files_[current_id_])};
        ++current_id_;

        return image;
    }

   private:
    std::vector<std::string> image_files_;
    std::size_t current_id_{0};
};

int main(int argc, char* argv[]) {
    char const* const filename{GetCommandOption(argv, argv + argc, "-c")};
    if (not filename) {
        std::cerr << "Target configuration yaml not provided! (-c <target_config_yaml>)" << std::endl;
        return EXIT_FAILURE;
    }

    YAML::Node const config{YAML::LoadFile(filename)};
    std::unique_ptr<feature_extraction::TargetExtractor> const extractor{
        feature_extraction::CreateTargetExtractor(config["target"])};

    auto image_feed{
        std::make_unique<FolderFeed>("data/data_fisheye_calibration/2013_05_29_drive_0000_extract/image_03/data_rgb")};

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