#pragma once

#include <opencv2/opencv.hpp>

#include "image_viewer/gui_interface.hpp"
#include "image_viewer/keyboard_input.hpp"

namespace reprojection::image_viewer {

// NOTE(Jack): What we are doing here with the GuiInterface and KeyboardInput is dependency injection. I went down this
// path because when I wanted to unit test the ImageViewer I could not without doing some magic behind the scenes.
// Therefore, I instead modified the class to take the gui and keyboard as dependencies which I could then mock in the
// unit test. Honestly it add some complexity and is very hardcoded/designed around the opencv style interface, but I
// think it is the right path going forward! If it gets annoying we can simply remove it and figure out how to test the
// image viewer another way :)

class ImageViewer {
   public:
    explicit ImageViewer(std::unique_ptr<GuiInterface> gui_interface, std::unique_ptr<KeyboardInput> keyboard_input,
                         int const delay_ms = 10);

    // NOTE(Jack): The cv::Mat is essentially a smart pointer with a reference counter - therefore we pass it here by
    // value so that the reference counter and not const& so the reference counter gets incremented and the memory will
    // not get deallocated.
    void Show(cv::Mat const frame);

    [[nodiscard]] bool ShouldQuit() const { return quit_; }

   private:
    void HandleInput();

    std::unique_ptr<GuiInterface> gui_interface_;
    std::unique_ptr<KeyboardInput> keyboard_input_;
    int delay_ms_;

    bool paused_{false};
    bool quit_{false};
};

}  // namespace reprojection::image_viewer