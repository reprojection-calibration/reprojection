#include "reprojection/reprojection.hpp"

#include <gtest/gtest.h>

#include <rosbag2_cpp/writer.hpp>

#include "testing_utils.hpp"

using namespace reprojection;

TEST(Ros2Application, TestSerializeBagTopic) {
    ros2::ScopedBagPath const temp_bag;
    {
        // The write needs to go out of scope before we can read from it again.
        rosbag2_cpp::Writer writer;
        writer.open(temp_bag.path);
        writer.write(ros2::DummyImage(), "/raw_image_topic", rclcpp::Time(1));
    }

    auto const reader_result{ros2::SingleTopicBagReader::Create(temp_bag.path, "/raw_image_topic")};
    ASSERT_TRUE(std::holds_alternative<ros2::SingleTopicBagReader>(reader_result));
    auto const& reader{std::get<ros2::SingleTopicBagReader>(reader_result)};

    auto result{ros2::SerializeBagTopic(reader)};
    ASSERT_TRUE(result.has_value());
    std::string const filename{std::filesystem::path(temp_bag.path).filename()};
    std::string const gt_result{filename + "_0.mcap|/raw_image_topic|1;|"};
    EXPECT_EQ(*result, gt_result);

    result = ros2::SerializeBagTopic(reader);
    EXPECT_FALSE(result.has_value());
}