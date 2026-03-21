#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "reprojection/bag_wrapper.hpp"
#include "reprojection/reprojection.hpp"

using namespace reprojection;

int main(int argc, char** argv) {
    ros::init(argc, argv, "no_name");

    try {
        auto const [bag_file, image_topic]{ros1::DummyLoadConfig()};
        ros1::BagWrapper const bag{bag_file, rosbag::bagmode::Read};

        rosbag::View view(bag.bag, rosbag::TopicQuery({image_topic}));
        for (auto const& msg : view) {
            std::cout << msg.getTime() << std::endl;
            std::cout << view.size() << std::endl;
        }
    } catch (rosbag::BagException& e) {
        ROS_ERROR("Error reading bag: %s", e.what());
        return 1;
    }

    return 0;

    return 0;
}