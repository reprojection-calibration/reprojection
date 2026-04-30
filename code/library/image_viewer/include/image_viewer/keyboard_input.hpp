#pragma once

#include <opencv2/opencv.hpp>

namespace reprojection::image_viewer {

class KeyboardInput {
   public:
    virtual ~KeyboardInput() = default;

    virtual int WaitKey(int delay) = 0;
};

class OpenCvKeyboardInput : public KeyboardInput {
   public:
    int WaitKey(int delay) override { return cv::waitKey(delay); }
};

}  // namespace reprojection::image_viewer