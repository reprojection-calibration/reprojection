#include <filesystem>

#include "application/reprojection_calibration.hpp"
#include "logging/logging.hpp"
#include "video_capture/video_capture.hpp"

using namespace reprojection;

namespace {

// We get a name conflict here with some math functions if we just use 'log' like we normally do, so prepend a
// underscore.
auto const _log{logging::Get("application")};

}  // namespace

int main(int argc, char* argv[]) {
    auto const app_args{application::ParseArgs(argc, argv)};
    if (not app_args) {
        return EXIT_FAILURE;
    }

    // TODO(Jack): How can we refactor the video file application to at least handle the stereo case? The main problem
    // is that we do not yet have a standard file/dataset structure for video file datasets that consist or more than
    // just one video file/image folder.
    application::Sensors const sensors{application::ParseSensors(app_args->config)};
    if (std::size(sensors.camera_names) > 1 or sensors.imu_name.has_value()) {
        _log->error(
            "{{'app': '{}', 'num_cameras': {}, 'imu': {}, 'msg': 'The video-file application is only suitable for "
            "monocular camera calibration (i.e. only '[cam0]'). Please remove any additional cameras or IMU from the "
            "configuration file.'}}",
            "video-file", std::size(sensors.camera_names), sensors.imu_name.has_value());

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

    ImageInputs const image_inputs{{sensors.camera_names.front(), {image_source, video_capture->GetSignature()}}};

    application::Calibrate(app_args->config, image_inputs, std::nullopt, app_args->db);

    return EXIT_SUCCESS;
}