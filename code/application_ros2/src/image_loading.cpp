#include "image_loading.hpp"

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace reprojection::ros2 {

cv::Mat ToCvMat(rosbag2_storage::SerializedBagMessage const& msg, std::string_view type) {
    rclcpp::SerializedMessage const serialized_msg(*msg.serialized_data);

    if (type == "sensor_msgs/msg/Image") {
        rclcpp::Serialization<sensor_msgs::msg::Image> serializer;
        sensor_msgs::msg::Image msg;
        serializer.deserialize_message(&serialized_msg, &msg);
        cv_bridge::CvImagePtr const cv_ptr{cv_bridge::toCvCopy(msg)};

        return cv_ptr->image;
    } else if (type == "sensor_msgs/msg/CompressedImage") {
        rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serializer;
        sensor_msgs::msg::CompressedImage msg;
        serializer.deserialize_message(&serialized_msg, &msg);
        cv_bridge::CvImagePtr const cv_ptr{cv_bridge::toCvCopy(msg)};

        return cv_ptr->image;
    } else {
        throw std::runtime_error("Failure during ROS2 image deserialization given type: " + std::string(type));
    }
}

}  // namespace reprojection::ros2
