#pragma once

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>

namespace reprojection::ros1 {

std_msgs::Header DummyHeader() {
    std_msgs::Header header;
    header.stamp = ros::Time(1);
    header.frame_id = "camera";

    return header;
}

sensor_msgs::Image DummyImage() {
    sensor_msgs::Image img_msg;
    img_msg.header = DummyHeader();

    cv::Mat const dummy{cv::Mat::zeros(1, 1, CV_8UC3)};
    img_msg.height = dummy.rows;
    img_msg.width = dummy.cols;
    img_msg.encoding = "bgr8";
    img_msg.is_bigendian = false;
    img_msg.step = dummy.step;
    img_msg.data.assign(dummy.data,
                        dummy.data + dummy.total() * dummy.elemSize());

    return img_msg;
}

sensor_msgs::CompressedImage DummyCompressedImage() {
    sensor_msgs::CompressedImage img_msg_comp;
    img_msg_comp.header = DummyHeader();

    // Build the minimum viable decodable image.
    cv::Mat const dummy{cv::Mat::zeros(1, 1, CV_8UC3)};
    std::vector<uchar> buffer;
    cv::imencode(".png", dummy, buffer);
    img_msg_comp.format = "png";
    img_msg_comp.data = buffer;

    return img_msg_comp;
}

}  // namespace reprojection::ros1