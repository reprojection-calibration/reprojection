#pragma once

#include <opencv2/opencv.hpp>

namespace reprojection::image_viewer {

class KeyboardInput {
   public:
    virtual ~KeyboardInput() = default;

    // WARN(Jack): I am not sure we are really ever gonna add another keyboard input provider besides opencv, but if we
    // do I want you to remember one thing. And that is this interface is here, getting the key and passing a delay,
    // seems very specific to opencv. I am not sure those things are really intimately related and if you are adding
    // another provider one day and realize that it does not make sense then think about redesigning the interface
    // before you
    virtual int WaitKey(int delay) = 0;
};

// LCOV_EXCL_START

class OpenCvKeyboardInput : public KeyboardInput {
   public:
    int WaitKey(int delay) override { return cv::waitKey(delay); }
};

// LCOV_EXCL_STOP

}  // namespace reprojection::image_viewer