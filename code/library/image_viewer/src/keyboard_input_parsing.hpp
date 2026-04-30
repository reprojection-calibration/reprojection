#pragma once

#include <optional>

namespace reprojection::image_viewer {

enum class KeyboardKey {
    EscapeKey = 27,
    LetterN = 110,
    LetterQ = 113,
    SpaceBar = 32,
};

std::optional<KeyboardKey> ToKeyboardKey(int const key);

}  // namespace reprojection::image_viewer