
#include "keyboard_input_parsing.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(ImageViewerKeyboardInputParsing, TestInputs) {
    auto result{image_viewer::ToKeyboardKey(27)};
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, image_viewer::KeyboardKey::EscapeKey);

    result = image_viewer::ToKeyboardKey(113);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, image_viewer::KeyboardKey::LetterQ);

    result = image_viewer::ToKeyboardKey(32);
    ASSERT_TRUE(result.has_value());
    EXPECT_EQ(*result, image_viewer::KeyboardKey::SpaceBar);

    result = image_viewer::ToKeyboardKey(666);
    EXPECT_FALSE(result.has_value());
}