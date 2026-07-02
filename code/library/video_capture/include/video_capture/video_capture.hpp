#pragma once

#include <string>

#include <opencv2/opencv.hpp>

namespace reprojection::video_capture {

class VideoCapture {
   public:
    explicit VideoCapture(int const device_id);

    // NOTE(Jack): We use the cv::VideoCapture api which supports both files (ex. mp4) but also folders with images in
    // them. Look at the opencv docs for more information.
    explicit VideoCapture(std::string const& video_file);

    ~VideoCapture();

    cv::Mat GetImage();

    std::string GetSignature();

   private:
    explicit VideoCapture(cv::VideoCapture const& cap);

    cv::VideoCapture cap_;
};

}  // namespace reprojection::video_capture