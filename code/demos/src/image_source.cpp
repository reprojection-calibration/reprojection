#include "demos/image_source.hpp"

#include <algorithm>
#include <filesystem>

namespace reprojection::demos {

// NOTE(Jack): We literally cannot unit test this because there is no way to simulate a physical camera device.
VideoCapture::VideoCapture(int const device_id)     // LCOV_EXCL_LINE
    : VideoCapture{cv::VideoCapture(device_id)} {}  // LCOV_EXCL_LINE

VideoCapture::VideoCapture(std::string const& video_file) : VideoCapture{cv::VideoCapture(video_file)} {}

VideoCapture::~VideoCapture() { cap_.release(); }

cv::Mat VideoCapture::GetImage() {
    // TODO(Jack): Formalize error handling strategy. If there are not more frames but we call cap_ >> frame I think it
    // just does nothing and we will return empty cv::Mat. The testing for the mp4 video based version shows me this
    // behavior, which I think is an OK policy. Not robust, but OK until we know better what to do.
    cv::Mat frame;
    cap_ >> frame;

    return frame;
}  // LCOV_EXCL_LINE

// Private constructor intended for internal use only.
VideoCapture::VideoCapture(cv::VideoCapture const& cap) : cap_{cap} {
    if (not cap_.isOpened()) {
        throw std::runtime_error("Video capture device is not open!");
    }
}

ImageFolder::ImageFolder(std::string const& image_folder) {
    std::ranges::for_each(std::filesystem::directory_iterator(image_folder),
                          [this](const auto& entry) { image_files_.push_back(entry.path()); });

    std::sort(std::begin(image_files_), std::end(image_files_));
}

cv::Mat ImageFolder::GetImage() {
    if (current_id_ >= std::size(image_files_)) {
        // Out of images to load, return empty, is this a valid way to handle this condition? Should we use optional
        // here to signal failure?
        return cv::Mat();
    }

    cv::Mat const image{cv::imread(image_files_[current_id_])};
    ++current_id_;

    return image;
}

}  // namespace reprojection::demos