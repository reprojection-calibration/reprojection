#include "keyboard_input_parsing.hpp"

namespace reprojection::image_viewer {

std::optional<KeyboardKey> ToKeyboardKey(int const key) {
    if (key == static_cast<int>(KeyboardKey::EscapeKey)) {
        return KeyboardKey::EscapeKey;
    }  else if (key == static_cast<int>(KeyboardKey::LetterN)) {
        return KeyboardKey::LetterN;
    } else if (key == static_cast<int>(KeyboardKey::LetterQ)) {
        return KeyboardKey::LetterQ;
    } else if (key == static_cast<int>(KeyboardKey::SpaceBar)) {
        return KeyboardKey::SpaceBar;
    } else {
        return std::nullopt;
    }
}

}  // namespace reprojection::image_viewer