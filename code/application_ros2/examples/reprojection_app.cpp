#include <rclcpp/rclcpp.hpp>

#include "reprojection/reprojection.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    reprojection::ros2::Delete();
    std::cout << "We are here ROS2" << std::endl;

    rclcpp::shutdown();
    return 0;
}