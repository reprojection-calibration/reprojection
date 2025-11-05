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

class VideoCaptureFeed : public ImageFeed {
   public:
    VideoCaptureFeed(int const device_id);

    VideoCaptureFeed(std::string const& video_file);

    ~VideoCaptureFeed() override;

    cv::Mat GetImage() override;

   private:
    cv::VideoCapture cap_;
};

class FolderFeed : public ImageFeed {
   public:
    FolderFeed(std::string const& image_folder);

    cv::Mat GetImage() override;

   private:
    std::vector<std::string> image_files_;
    std::size_t current_id_{0};
};

}  // namespace reprojection::demos