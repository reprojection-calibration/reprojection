#include "reprojection/reprojection.hpp"

#include <gtest/gtest.h>

#include <filesystem>

#include "reprojection/bag_wrapper.hpp"

#include "testing_utils.hpp"

using namespace reprojection;

TEST(Ros1Reprojection, TestTopicCacheString) {
    ros1::ScopedBagPath const temp_bag;

    // Cannot get cache string from a write mode bag (rosbag::View needs a bag to be in read mode).
    ros1::BagWrapper write_handle(temp_bag.path, rosbag::bagmode::Write);
    auto result{ros1::TopicCacheString(write_handle, "")};
    EXPECT_FALSE(result.has_value());

    {
        ros1::BagWrapper write_handle_1(temp_bag.path, rosbag::bagmode::Write);
        write_handle_1.bag.write("/raw_image_topic", ros::Time(1), ros1::DummyImage());
    }

    ros1::BagWrapper read_handle(temp_bag.path, rosbag::bagmode::Read);
    result = ros1::TopicCacheString(read_handle, "/raw_image_topic");
    ASSERT_TRUE(result.has_value());

    std::string const filename{std::filesystem::path(temp_bag.path).filename().c_str()};
    std::string const gt_result{filename + "|/raw_image_topic|1|1.000;|"};
    EXPECT_EQ(*result, gt_result);

    // Check that a topic not found in the bag returns std::nullopt.
    result = ros1::TopicCacheString(read_handle, "/nonexistent_topic");
    EXPECT_FALSE(result.has_value());
}