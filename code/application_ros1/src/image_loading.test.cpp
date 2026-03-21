#include "image_loading.hpp"

#include <gtest/gtest.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "testing_utils.hpp"

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