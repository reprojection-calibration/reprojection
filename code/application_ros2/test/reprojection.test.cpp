#include "reprojection/reprojection.hpp"

#include <gtest/gtest.h>

using namespace reprojection;

TEST(Ros2Application, TestXxx) {
    auto const [bag_file, image_topic]{ros2::DummyLoadConfig()};  // REMOVE

    EXPECT_EQ(1, 1);
}