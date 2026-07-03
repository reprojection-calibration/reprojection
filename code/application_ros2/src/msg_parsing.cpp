#include "msg_parsing.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace reprojection::ros2 {

uint64_t GetTimestampNs(std_msgs::msg::Header const& header) {
    return static_cast<uint64_t>(header.stamp.sec) * 1'000'000'000ULL + static_cast<uint64_t>(header.stamp.nanosec);
}

std::pair<uint64_t, cv::Mat> ToCvMat(rosbag2_storage::SerializedBagMessage const& bag_msg, std::string_view type) {
    rclcpp::SerializedMessage const serialized_msg(*bag_msg.serialized_data);

    if (type == "sensor_msgs/msg/Image") {
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        sensor_msgs::msg::Image msg;
        serializer.deserialize_message(&serialized_msg, &msg);
        cv_bridge::CvImagePtr const cv_ptr{cv_bridge::toCvCopy(msg)};

        uint64_t const timestamp_ns{GetTimestampNs(msg.header)};

        return {timestamp_ns, cv_ptr->image};
    } else if (type == "sensor_msgs/msg/CompressedImage") {
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
        sensor_msgs::msg::CompressedImage msg;
        serializer.deserialize_message(&serialized_msg, &msg);
        cv_bridge::CvImagePtr const cv_ptr{cv_bridge::toCvCopy(msg)};

        uint64_t const timestamp_ns{GetTimestampNs(msg.header)};

        return {timestamp_ns, cv_ptr->image};
    } else {
        throw std::runtime_error("Failure during ROS2 image deserialization given type: " + std::string(type));
    }
}

std::pair<uint64_t, std::array<double, 6>> ToImuArray(rosbag2_storage::SerializedBagMessage const& bag_msg) {
    rclcpp::SerializedMessage const serialized_msg(*bag_msg.serialized_data);

    // TODO(Jack): Do we need to put this into a try catch block?
    rclcpp::Serialization<sensor_msgs::msg::Imu> serializer;
    sensor_msgs::msg::Imu msg;
    serializer.deserialize_message(&serialized_msg, &msg);

    std::array<double, 6> const imu_data{
        msg.angular_velocity.x,    msg.angular_velocity.y,    msg.angular_velocity.z,
        msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
    };
    uint64_t const timestamp_ns{GetTimestampNs(msg.header)};

    return {timestamp_ns, imu_data};
}

}  // namespace reprojection::ros2
