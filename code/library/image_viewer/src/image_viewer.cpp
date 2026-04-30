#include "image_viewer/image_viewer.hpp"

#include "keyboard_input_parsing.hpp"

namespace reprojection::image_viewer {

ImageViewer::ImageViewer(std::unique_ptr<GuiInterface> gui_interface, std::unique_ptr<KeyboardInput> keyboard_input,
                         int const delay_ms)
    : gui_interface_{std::move(gui_interface)}, keyboard_input_{std::move(keyboard_input)}, delay_ms_{delay_ms} {
    gui_interface_->OpenWindow();
}

void ImageViewer::Show(cv::Mat const frame) {
    if (frame.empty()) {
        return;  // LCOV_EXCL_LINE
    }

    gui_interface_->Show(frame);

    this->HandleInput();
}

void ImageViewer::HandleInput() {
    int const input{keyboard_input_->WaitKey(paused_ ? 0 : delay_ms_)};

    auto const key{ToKeyboardKey(input)};
    if (not key) {
        return;  // LCOV_EXCL_LINE
    }

    if (*key == KeyboardKey::EscapeKey or *key == KeyboardKey::LetterQ) {
        quit_ = true;
    } else if (*key == KeyboardKey::SpaceBar) {  // LCOV_EXCL_LINE
        paused_ = not paused_;                   // LCOV_EXCL_LINE
    } else {
        throw std::runtime_error{
            "LIBRARY IMPLEMENTATION ERROR - unimplemented keyboard input handling"};  // LCOV_EXCL_LINE
    }
}

}  // namespace reprojection::image_viewer