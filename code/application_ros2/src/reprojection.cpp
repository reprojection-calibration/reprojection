#include "reprojection/reprojection.hpp"

namespace reprojection::ros2 {

std::tuple<std::string, std::string> DummyLoadConfig() {
    return {"/data/TUM-Visual-Inertial-Dataset/dataset-calib-imu4.bag", "/cam0/image_raw"};
}

}  // namespace reprojection::ros2
