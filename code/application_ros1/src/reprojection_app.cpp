#include <ros/ros.h>

#include "reprojection/reprojection.hpp"

int main(int argc, char** argv) {
    ros::init(argc, argv, "no_name");

    reprojection::ros1::Delete();
    std::cout << "We are here ROS1" << std::endl;

    return 0;
}