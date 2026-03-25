#include "reprojection/reprojection.hpp"

#include <gtest/gtest.h>

#include "reprojection/bag_wrapper.hpp"

#include "testing_utils.hpp"

using namespace reprojection;

TEST(Ros1Reprojection, TestSerializeBagTopic) {
    ros1::ScopedBagPath const temp_bag;
    {
        rosbag::Bag bag;
        bag.open(temp_bag.path, rosbag::bagmode::Write);
        bag.write("/raw_image_topic", ros::Time(1), ros1::DummyImage());
        bag.close();
    }

    auto const reader_result{ros1::SingleTopicBagReader::Create(temp_bag.path, "/raw_image_topic")};
    ASSERT_TRUE(std::holds_alternative<ros1::SingleTopicBagReader>(reader_result));
    auto const& reader{std::get<ros1::SingleTopicBagReader>(reader_result)};

    auto result{ros1::SerializeBagTopic(reader)};
    ASSERT_TRUE(result.has_value());
    std::string const filename{std::filesystem::path(temp_bag.path).filename()};
    std::string const gt_result{filename + "|/raw_image_topic|1.000;|"};
    EXPECT_EQ(*result, gt_result);
}