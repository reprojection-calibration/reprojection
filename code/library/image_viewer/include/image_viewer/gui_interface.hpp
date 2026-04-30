#pragma once

#include <opencv2/opencv.hpp>

namespace reprojection::image_viewer {

class GuiInterface {
   public:
    GuiInterface(std::string_view window_name) : window_name_{window_name} {}

    virtual ~GuiInterface() = default;

    virtual void OpenWindow() = 0;

    virtual void Show(cv::Mat const frame) = 0;

   protected:
    std::string window_name_;
};

// NOTE(Jack): Because of the headless nature of CI pipelines there is no reasonable way to unit test the gui or
// keyboard interface components.

// LCOV_EXCL_START

class OpenCvGuiInterface : public GuiInterface {
   public:
    OpenCvGuiInterface(std::string_view window_name) : GuiInterface(window_name) {}

    void OpenWindow() override { cv::namedWindow(window_name_, cv::WINDOW_NORMAL); }

    void Show(cv::Mat const frame) override { cv::imshow(window_name_, frame); }
};

// LCOV_EXCL_STOP

}  // namespace reprojection::image_viewer