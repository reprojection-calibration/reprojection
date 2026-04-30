#include "image_viewer/image_viewer.hpp"

#include "keyboard_input_parsing.hpp"

namespace reprojection::image_viewer {

ImageViewer::ImageViewer(std::string_view const& window_name, int const delay_ms)
    : window_name_{window_name}, delay_ms_{delay_ms} {
    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
}

void ImageViewer::Show(cv::Mat const frame) {
    if (frame.empty()) {
        return;
    }

    cv::imshow(window_name_, frame);

    HandleInput();
}

void ImageViewer::HandleInput() {
    int const keyboard_input{cv::waitKey(paused_ ? 0 : delay_ms_)};

    auto const key{ToKeyboardKey(keyboard_input)};
    if (not key) {
        return;
    }

    if (*key == KeyboardKey::EscapeKey or *key == KeyboardKey::LetterQ) {
        quit_ = true;
    } else if (*key == KeyboardKey::SpaceBar) {
        paused_ = not paused_;
    } else {
        throw std::runtime_error{"LIBRARY IMPLEMENTATION ERROR - unimplemented keyboard input handling"};
    }
}

}  // namespace reprojection::image_viewer