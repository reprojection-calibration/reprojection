#include <gtest/gtest.h>

#include <cv_bridge/cv_bridge.h>
#include <rosbag/bag.h>
#include <rosbag/message_instance.h>
#include <rosbag/view.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>

#include "testing_utils.hpp"

namespace reprojection::ros1 {

cv::Mat ToCvMat(rosbag::MessageInstance const& msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    if (msg.getDataType() == "sensor_msgs/Image") {
        sensor_msgs::Image::ConstPtr const img_msg{msg.instantiate<sensor_msgs::Image>()};
        cv_ptr = cv_bridge::toCvCopy(img_msg);
    } else if (msg.getDataType() == "sensor_msgs/CompressedImage") {
        sensor_msgs::CompressedImage::ConstPtr const img_msg{msg.instantiate<sensor_msgs::CompressedImage>()};
        cv_ptr = cv_bridge::toCvCopy(img_msg);
    } else {
        throw std::runtime_error("You asked to decode a non-image message into an image, what's going on?");
    }

    return cv_ptr->image;
}

}  // namespace reprojection::ros1

using namespace reprojection;

// TODO UNIFY BAG OPENING/CLOSING AND GENERAL HANDLING!
TEST(Ros1ImageLoading, TestToCvMat) {
    rosbag::Bag bag;
    bag.open("/tmp/delete.bag", rosbag::bagmode::Write);
    bag.write("/raw_image_topic", ros::Time(1), ros1::DummyImage());
    bag.write("/compressed_image_topic", ros::Time(1), ros1::DummyCompressedImage());
    bag.close();

    bag.open("/tmp/delete.bag", rosbag::bagmode::Read);
    rosbag::View view(bag);
    for (auto const& msg : view) {
        cv::Mat image;
        EXPECT_NO_THROW(image = ros1::ToCvMat(msg));

        EXPECT_EQ(image.rows, 1);
        EXPECT_EQ(image.cols, 1);
    }

    bag.close();

    EXPECT_EQ(1, 1);
}