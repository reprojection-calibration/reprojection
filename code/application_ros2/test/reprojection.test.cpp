#include "application_ros2/reprojection.hpp"

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
}

TEST(Ros2Application, TestImageSource) {
    ros2::ScopedBagPath const temp_bag;
    {
        // The write needs to go out of scope before we can read from it again.
        rosbag2_cpp::Writer writer;
        writer.open(temp_bag.path);
        writer.write(ros2::DummyImage(rclcpp::Time(1)), "/raw_image_topic", rclcpp::Time(1));
        writer.write(ros2::DummyImage(rclcpp::Time(2)), "/raw_image_topic", rclcpp::Time(2));
    }

    // TODO(Jack): The const semantics of the ROS2 image source are not so clear when compared to the ROS1 version. It
    // would be nice to align there two if we can find a consistent way to allow the ROS2 bag reader to be passed in as
    // const.
    auto reader_result{ros2::SingleTopicBagReader::Create(temp_bag.path, "/raw_image_topic")};
    ASSERT_TRUE(std::holds_alternative<ros2::SingleTopicBagReader>(reader_result));
    auto& reader{std::get<ros2::SingleTopicBagReader>(reader_result)};

    ros2::ImageSource image_source{reader};

    auto data{image_source()};
    ASSERT_TRUE(data.has_value());
    auto [timestamp_ns, img]{*data};
    EXPECT_EQ(timestamp_ns, 1);

    data = image_source();
    ASSERT_TRUE(data.has_value());
    std::tie(timestamp_ns, img) = *data;
    EXPECT_EQ(timestamp_ns, 2);

    data = image_source();
    EXPECT_FALSE(data.has_value());
}