#include "image_loading.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

namespace reprojection::ros1 {

// TODO RETURN DIRECTLY IN IF ELSE BLOCK!
std::pair<uint64_t, cv::Mat> ToCvMat(rosbag::MessageInstance const& msg) {
    if (msg.getDataType() == "sensor_msgs/Image") {
        sensor_msgs::Image::ConstPtr const img_msg{msg.instantiate<sensor_msgs::Image>()};
        cv_bridge::CvImageConstPtr const cv_ptr{cv_bridge::toCvCopy(img_msg)};

        uint64_t const timestamp_ns{img_msg->header.stamp.toNSec()};

        return {timestamp_ns, cv_ptr->image};
    }

    if (msg.getDataType() == "sensor_msgs/CompressedImage") {
        sensor_msgs::CompressedImage::ConstPtr const img_msg{msg.instantiate<sensor_msgs::CompressedImage>()};

        cv_bridge::CvImageConstPtr const cv_ptr{cv_bridge::toCvCopy(img_msg)};
        uint64_t const timestamp_ns{img_msg->header.stamp.toNSec()};

        return {timestamp_ns, cv_ptr->image};
    }

    throw std::runtime_error("You asked to decode a non-image message into an image, what's going on?");
}

std::pair<uint64_t, std::array<double, 6>> ToImuArray(rosbag::MessageInstance const& msg) {
    if (msg.getDataType() == "sensor_msgs/Imu") {
        sensor_msgs::Imu::ConstPtr const imu_msg{msg.instantiate<sensor_msgs::Imu>()};

        std::array<double, 6> const imu_data{
            imu_msg->angular_velocity.x,    imu_msg->angular_velocity.y,    imu_msg->angular_velocity.z,
            imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z,
        };
        uint64_t const timestamp_ns{imu_msg->header.stamp.toNSec()};

        return {timestamp_ns, imu_data};
    } else {
        throw std::runtime_error("You asked to decode a non-imu message into an imu array, what's going on?");
    }
}

}  // namespace reprojection::ros1
