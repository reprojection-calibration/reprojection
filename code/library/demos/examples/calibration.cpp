#include <chrono>
#include <filesystem>

#include "application/reprojection_calibration.hpp"
#include "demos/image_source.hpp"

namespace ch = std::chrono;
namespace fs = std::filesystem;
using namespace reprojection;

int main(int argc, char* argv[]) {
    auto const app_args{application::ParseArgs(argc, argv)};
    if (not app_args) {
        return EXIT_FAILURE;
    }

    // TODO(Jack): At this time we only support video files (ex. .mp4). It would be nice to also support video devices
    // (i.e. webcams) and potentially also folders of images. Whatever we do we need to make sure to respect the
    // semantics of application::ParseArgs() and unify it with the code in the feature extraction demo. The
    // demos::ImageSource already brings us a lot of the way there I think but it needs some more engineering to reach
    // the above goals.
    // TODO(Jack): Unify the demos::ImageSource with the application/types ImageSource
    std::unique_ptr<demos::ImageSource> const image_feed{std::make_unique<demos::VideoCapture>(app_args->data_path)};

    ImageSource image_source{[&image_feed]() -> std::optional<std::pair<uint64_t, cv::Mat>> {
        auto const timestamp_ns{ch::duration_cast<ch::nanoseconds>(ch::steady_clock::now().time_since_epoch()).count()};

        cv::Mat const img{image_feed->GetImage()};
        if (img.empty()) {
            return std::nullopt;
        }

        std::cout << "got image " << img.size << std::endl;
        return std::pair<uint64_t, cv::Mat>{timestamp_ns, img};
    }};

    application::Calibrate(app_args->config, image_source, "", app_args->db);

    return EXIT_SUCCESS;
}