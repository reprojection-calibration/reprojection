#include "image_viewer/image_viewer.hpp"

#include <gtest/gtest.h>

#include "image_viewer/keyboard_input.hpp"

using namespace reprojection;

namespace reprojection::image_viewer {

// NOTE(Jack): We hardcode the escape key here because the quitting behavior is the only state change that we can
// query from public methods. The pause/step through behavior is not testable from public methods therefore we
// do not even try.
class MockedKeyboardInput : public KeyboardInput {
   public:
    int WaitKey(int delay) override {
        static_cast<void>(delay);

        return escape_key;
    }

    int escape_key{27};
};

}  // namespace reprojection::image_viewer

TEST(ImageViewerImageViewer, TestXxx) {
    image_viewer::ImageViewer viewer(std::make_unique<image_viewer::MockedKeyboardInput>());

    EXPECT_FALSE(viewer.ShouldQuit());

    // Keyboard input is only queried at the end of the Show() method. Therefore after this point it will register the
    // escape key and ShouldQuit() will be true.
    EXPECT_NO_THROW(viewer.Show(cv::Mat::ones(1, 3, CV_8UC3)));
    EXPECT_TRUE(viewer.ShouldQuit());
}