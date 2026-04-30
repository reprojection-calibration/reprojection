#pragma once

#include <optional>

namespace reprojection::image_viewer {

// WARN(Jack): I got these from my asus laptop running ubuntu 24.04 - if these really are generic and apply elsewhere I
// am not 100% sure yet.
enum class KeyboardKey {
    EscapeKey = 27,
    LetterQ = 113,
    SpaceBar = 32,
};

std::optional<KeyboardKey> ToKeyboardKey(int const key);

}  // namespace reprojection::image_viewer