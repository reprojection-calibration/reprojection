#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace reprojection::demos {

class ImageSource {
   public:
    virtual ~ImageSource() = default;

    virtual cv::Mat GetImage() = 0;
};

class VideoCapture final : public ImageSource {
   public:
    explicit VideoCapture(int const device_id);

    explicit VideoCapture(std::string const& video_file);

    ~VideoCapture() override;

    cv::Mat GetImage() override;

   private:
    explicit VideoCapture(cv::VideoCapture const& cap);

    cv::VideoCapture cap_;
};

class ImageFolder final : public ImageSource {
   public:
    explicit ImageFolder(std::string const& image_folder);

    cv::Mat GetImage() override;

   private:
    std::vector<std::string> image_files_;
    std::size_t current_id_{0};
};

}  // namespace reprojection::demos