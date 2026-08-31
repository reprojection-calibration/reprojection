#include <filesystem>

#include "application/reprojection_calibration.hpp"
#include "video_capture/video_capture.hpp"

using namespace reprojection;

int main(int argc, char* argv[]) {
    auto const app_args{application::ParseArgs(argc, argv)};
    if (not app_args) {
        return EXIT_FAILURE;
    }

    // TODO(Jack): How can we refactor the video file application to at least handle the stereo case?
    application::Sensors const sensors{application::ParseSensors(app_args->config)};
    if (std::size(sensors.camera_names) or sensors.imu_name.has_value()) {
        std::cerr << std::format(
            "The video file application only does monocular camera calibration - you passed a configuration for with "
            "{} cameras and a imu is present: {}",
            std::size(sensors.camera_names), sensors.imu_name.has_value());
        return EXIT_FAILURE;
    }

    auto const video_capture{std::make_unique<video_capture::VideoCapture>(app_args->data_path)};

    // NOTE(Jack): We use a simple incremented timestamp here because we have no easily accessible time information from
    // the the opencv video reader interface we use. Does anyone know if that is maybe not the case and we can get time
    // data?
    int pseudo_timestamp{0};
    ImageSampler image_source{[&video_capture, &pseudo_timestamp]() -> std::optional<std::pair<uint64_t, cv::Mat>> {
        cv::Mat img{video_capture->GetImage()};
        if (img.empty()) {
            return std::nullopt;
        }

        return std::pair<uint64_t, cv::Mat>{pseudo_timestamp++, img};
    }};

    application::ImageInputs const image_inputs{{"video_file", {image_source, video_capture->GetSignature()}}};
    application::Calibrate(app_args->config, image_inputs, std::nullopt, app_args->db);

    return EXIT_SUCCESS;
}