#include "reprojection/reprojection.hpp"

#include <rclcpp/rclcpp.hpp>

namespace reprojection::ros2 {

void Delete() { return; }

}  // namespace reprojection::ros2

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    reprojection::ros2::Delete();
    std::cout << "We are here" << std::endl;

    rclcpp::shutdown();
    return 0;
}