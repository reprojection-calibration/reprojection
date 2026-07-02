#include "application/image_source.hpp"

#include <algorithm>
#include <filesystem>

namespace reprojection::application {

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

std::string VideoCapture::GetSignature() {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    // WARN(Jack): Opencv itself warns that "Effective behavior depends from device driver and API Backend" for the
    // .get() api used here. With that in mind I think it would be optimistic to expect for the signature to be the same
    // on a different computer, but for a single computer the collection of following information should hopefully
    // provide a unique signature which can be used to create the step cache key.
    oss << cap_.get(cv::CAP_PROP_POS_MSEC) << "|";
    oss << cap_.get(cv::CAP_PROP_POS_FRAMES) << "|";
    oss << cap_.get(cv::CAP_PROP_FRAME_WIDTH) << "|";
    oss << cap_.get(cv::CAP_PROP_FRAME_HEIGHT) << "|";
    oss << cap_.get(cv::CAP_PROP_FPS) << "|";
    oss << cap_.get(cv::CAP_PROP_FOURCC) << "|";
    oss << cap_.get(cv::CAP_PROP_FRAME_COUNT) << "|";
    oss << "(" << cap_.get(cv::CAP_PROP_BACKEND) << ", " << cap_.getBackendName() << ")" << "|";

    return oss.str();
}

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

std::string ImageFolder::GetSignature() {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    // TODO(Jack): We should use the name of all images files for the signature! Not just the first!
    oss << std::size(image_files_) << "|";
    oss << image_files_[0] << "|";

    return oss.str();
}

}  // namespace reprojection::application