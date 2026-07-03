#include <filesystem>

#include "application/reprojection_calibration.hpp"
#include "video_capture/video_capture.hpp"

namespace fs = std::filesystem;
using namespace reprojection;

int main(int argc, char* argv[]) {
    auto const app_args{application::ParseArgs(argc, argv)};
    if (not app_args) {
        return EXIT_FAILURE;
    }

    auto const video_capture{std::make_unique<video_capture::VideoCapture>(app_args->data_path)};

    // NOTE(Jack): We use a simple incremented timestamp here because we have no easily accessible time information from
    // the video file (at least I don't think we can get that). I had first planned to use a system timestamp using
    // the chrono library but then that meant the integration testing would not work because then the cache key would
    // change every time.
    int pseudo_timestamp{0};
    ImageSampler image_source{[&video_capture, &pseudo_timestamp]() -> std::optional<std::pair<uint64_t, cv::Mat>> {
        cv::Mat img{video_capture->GetImage()};
        if (img.empty()) {
            return std::nullopt;
        }

        return std::pair<uint64_t, cv::Mat>{pseudo_timestamp++, img};
    }};

    application::Calibrate(app_args->config, {image_source, video_capture->GetSignature()}, std::nullopt, app_args->db);

    return EXIT_SUCCESS;
}