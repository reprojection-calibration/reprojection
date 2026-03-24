#include "reprojection/reprojection.hpp"

#include <gtest/gtest.h>

#include <filesystem>

#include "reprojection/bag_wrapper.hpp"

#include "testing_utils.hpp"

using namespace reprojection;

TEST(Ros1Reprojection, TestSerializeBagTopic) {
    ros1::ScopedBagPath const temp_bag;
    {
        ros1::BagWrapper write_handle_1(temp_bag.path, rosbag::bagmode::Write);
        write_handle_1.bag.write("/raw_image_topic", ros::Time(1), ros1::DummyImage());
    }

    ros1::SingleTopicBagReader read_handle(temp_bag.path, "/raw_image_topic");

    auto result{ros1::SerializeBagTopic(read_handle)};
    ASSERT_TRUE(result.has_value());
    std::string const filename{std::filesystem::path(temp_bag.path).filename()};
    std::string const gt_result{filename + "|/raw_image_topic|1.000;|"};
    EXPECT_EQ(*result, gt_result);
}