#include "image_loading.hpp"

#include <gtest/gtest.h>

#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>

#include "testing_utils.hpp"

using namespace reprojection;

TEST(Ros2Application, TestImageLoading) {
    ros2::ScopedBagPath const temp_bag;
    {
        // The write needs to go out of scope before we can read from it again.
        rosbag2_cpp::Writer writer;
        writer.open(temp_bag.path);
        writer.write(ros2::DummyImage(), "/raw_image_topic", rclcpp::Time(1));
        writer.write(ros2::DummyCompressedImage(), "/compressed_image_topic", rclcpp::Time(1));
    }

    rosbag2_cpp::Reader reader;
    reader.open(std::string(temp_bag.path));

    // TODO(Jack): Annoyingly the ROS2 bag API provides NO way to get the type of a message directly. In ROS1 you
    // can
    //  just query the type from the serialized message and then decode it. In ROS2 we need to first construct this
    //  topic to type mapping from the reader metadata, and then use the topic name which is available before
    //  deserialization, to decide how to deserialize it. There has to be a better way to do this...
    std::unordered_map<std::string, std::string> topic_type_map;
    for (auto const& topic : reader.get_all_topics_and_types()) {
        topic_type_map[topic.name] = topic.type;
    }

    EXPECT_EQ(reader.get_metadata().message_count, 2);
    while (reader.has_next()) {
        auto const msg{reader.read_next()};

        cv::Mat img;
        EXPECT_NO_THROW(img = ros2::ToCvMat(*msg, topic_type_map[msg->topic_name]));
        EXPECT_EQ(img.rows, 1);
        EXPECT_EQ(img.cols, 1);
    }
}