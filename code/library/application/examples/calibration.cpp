#include <chrono>
#include <filesystem>

#include "application/image_source.hpp"
#include "application/reprojection_calibration.hpp"

namespace ch = std::chrono;
namespace fs = std::filesystem;
using namespace reprojection;

int main(int argc, char* argv[]) {
    auto const app_args{application::ParseArgs(argc, argv)};
    if (not app_args) {
        return EXIT_FAILURE;
    }

    // TODO(Jack): At this time we only support video files (ex. .mp4). It would be nice to also support video devices
    // (i.e. webcams) and potentially also folders of images (i.e. the entire VideoCapture api we expose). Whatever we
    // do we need to make sure to respect the semantics of application::ParseArgs() and unify it with the code in the
    // feature extraction demo. The application::ImageSource already brings us a lot of the way there I think
    // but it needs some more engineering to reach the above goals - for example how would we cache images or show live
    // feature extraction given the fact that we first write the images to the database and then do the extraction?
    // TODO(Jack): Unify the application::ImageSource with the types ImageSourceSignature
    std::unique_ptr<application::ImageSource> const image_feed{
        std::make_unique<application::VideoCapture>(app_args->data_path)};

    // NOTE(Jack): We use a simple incremented timestamp here because we have no easily accessible time information from
    // the video file (at least I don't think we can get that). I had first planned to use a system timestamp using
    // the chrono library but then that meant the integration testing would not work because then the cache key would
    // change every time.
    int pseudo_timestamp{0};
    ImageSourceSignature image_source{
        [&image_feed, &pseudo_timestamp]() -> std::optional<std::pair<uint64_t, cv::Mat>> {
            cv::Mat img{image_feed->GetImage()};
            if (img.empty()) {
                return std::nullopt;
            }

            return std::pair<uint64_t, cv::Mat>{pseudo_timestamp++, img};
        }};

    application::Calibrate(app_args->config, image_source, "", app_args->db);

    std::cout << "The future is calibrated!\n";
    return EXIT_SUCCESS;
}