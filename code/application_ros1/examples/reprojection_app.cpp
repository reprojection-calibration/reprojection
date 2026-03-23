#include <reprojection/application/load_and_validate_config.hpp>

#include "reprojection/bag_wrapper.hpp"
#include "reprojection/reprojection.hpp"

using namespace reprojection;

int main() {
    try {
        auto const [bag_file, image_topic]{ros1::DummyLoadConfig()};
        ros1::BagWrapper const bag{bag_file, rosbag::bagmode::Read};

        auto const cache_string{TopicCacheString(bag, image_topic)};
        std::cout << cache_string.value() << std::endl;
    } catch (rosbag::BagException& e) {
        return 1;
    }

    return 0;
}