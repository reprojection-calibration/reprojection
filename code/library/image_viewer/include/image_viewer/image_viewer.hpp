#pragma once

#include <deque>

#include <opencv2/opencv.hpp>

namespace reprojection::image_viewer {

class ImageViewer {
   public:
    explicit ImageViewer(std::string_view const& window_name = "Image Viewer", int const delay_ms = 30);

    // NOTE(Jack): The cv::Mat is essentially a smart pointer with a reference counter - therefore we pass it here by
    // value so that the reference counter and not const& so the reference counter gets incremented and the memory will
    // not get deallocated.
    void Show(cv::Mat const frame);

    bool ShouldQuit() const { return quit_; }

   private:
    void HandleInput();

    std::string window_name_;
    int delay_ms_;

    bool paused_{false};
    bool quit_{false};
};

}  // namespace reprojection::image_viewer