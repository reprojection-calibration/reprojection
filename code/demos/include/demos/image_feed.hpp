#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace reprojection::demos {

class ImageFeed {
   public:
    virtual ~ImageFeed() = default;

    virtual cv::Mat GetImage() = 0;
};

class VideoCapture final : public ImageFeed {
   public:
    explicit VideoCapture(int const device_id);

    explicit VideoCapture(std::string const& video_file);

    ~VideoCapture() override;

    cv::Mat GetImage() override;

   private:
    explicit VideoCapture(cv::VideoCapture const& cap);

    cv::VideoCapture cap_;
};

class ImageFolder final : public ImageFeed {
   public:
    explicit ImageFolder(std::string const& image_folder);

    cv::Mat GetImage() override;

   private:
    std::vector<std::string> image_files_;
    std::size_t current_id_{0};
};

}  // namespace reprojection::demos