#pragma once

#include <opencv2/opencv.hpp>

namespace reprojection::image_viewer {

class KeyboardInput {
   public:
    virtual ~KeyboardInput() = default;

    virtual int WaitKey(int delay) = 0;
};

// LCOV_EXCL_START

class OpenCvKeyboardInput : public KeyboardInput {
   public:
    int WaitKey(int delay) override { return cv::waitKey(delay); }
};

// LCOV_EXCL_STOP

}  // namespace reprojection::image_viewer