#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace reprojection::application {

class ImageSource {
   public:
    virtual ~ImageSource() = default;

    virtual cv::Mat GetImage() = 0;

    virtual std::string GetSignature() = 0;
};

class VideoCapture final : public ImageSource {
   public:
    explicit VideoCapture(int const device_id);

    explicit VideoCapture(std::string const& video_file);

    ~VideoCapture() override;

    cv::Mat GetImage() override;

    std::string GetSignature() override;

   private:
    explicit VideoCapture(cv::VideoCapture const& cap);

    cv::VideoCapture cap_;
};


}  // namespace reprojection::application