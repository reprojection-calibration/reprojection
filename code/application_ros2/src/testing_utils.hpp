#pragma once

#include <filesystem>
#include <random>
#include <string>

#include <opencv2/opencv.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

namespace reprojection::ros2 {

// TODO(Jack): All these dummy datatypes are passed almost identically from the ROS1 code. Is there a non intrusive strategy to reduce copy paste? Or do we even need these here?
inline std_msgs::msg::Header DummyHeader() {
    std_msgs::msg::Header header;
    header.stamp = rclcpp::Time(1);
    header.frame_id = "camera";

    return header;
}

inline sensor_msgs::msg::Image DummyImage() {
    sensor_msgs::msg::Image img_msg;
    img_msg.header = DummyHeader();

    cv::Mat const dummy{cv::Mat::zeros(1, 1, CV_8UC3)};
    img_msg.height = dummy.rows;
    img_msg.width = dummy.cols;
    img_msg.encoding = "bgr8";
    img_msg.is_bigendian = false;
    img_msg.step = dummy.step;
    img_msg.data.assign(dummy.data, dummy.data + dummy.total() * dummy.elemSize());

    return img_msg;
}

inline sensor_msgs::msg::CompressedImage DummyCompressedImage() {
    sensor_msgs::msg::CompressedImage img_msg_comp;
    img_msg_comp.header = DummyHeader();

    // Build the minimum viable decodable image.
    cv::Mat const dummy{cv::Mat::zeros(1, 1, CV_8UC3)};
    std::vector<uchar> buffer;
    cv::imencode(".png", dummy, buffer);
    img_msg_comp.format = "png";
    img_msg_comp.data = buffer;

    return img_msg_comp;
}

// ERROR COPY AND PASTED DIRECTLY FROM THE ROS1 APP! Also we have a very similar logic in the "temp file" testing util
// stuff in the library. We need a uniform handling of this topic. Maybe even install the testing utils...?
class ScopedBagPath {
   public:
    ScopedBagPath() : path{GeneratePath()} {}

    ~ScopedBagPath() { std::filesystem::remove(path); }

    std::string path;

   private:
    static std::string GeneratePath() {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::uniform_int_distribution<uint64_t> dis;

        std::filesystem::path const tmp{std::filesystem::temp_directory_path()};
        std::string const filename{"rosbag_test_" + std::to_string(dis(gen)) + ".db3"};

        return (tmp / filename).string();
    }
};

}  // namespace reprojection::ros2