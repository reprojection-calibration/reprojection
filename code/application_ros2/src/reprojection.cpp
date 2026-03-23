#include "reprojection/reprojection.hpp"

#include <filesystem>
#include <iostream>

#include <rosbag2_cpp/reader.hpp>

namespace reprojection::ros2 {

// TEST!!!!!
// TEST!!!!!
// TEST!!!!!
// TEST!!!!!
// TEST!!!!!
std::optional<std::string> SerializeBagTopic(std::string_view bag_path, std::string_view topic) {
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = std::string(bag_path);
    storage_options.storage_id = "mcap";

    rosbag2_cpp::ConverterOptions const converter_options{"cdr", "cdr"};

    rosbag2_cpp::Reader reader;
    reader.open(storage_options, converter_options);

    rosbag2_storage::StorageFilter filter;
    filter.topics = {std::string(topic)};
    reader.set_filter(filter);

    std::ostringstream oss;
    oss << std::filesystem::path(bag_path).filename().c_str() << "|";
    oss << topic << "|";

    int count{0};
    while (reader.has_next()) {
        auto const msg{reader.read_next()};
        //  NOTE(Jack): This returns a long int representing the time in nanoseconds. In ROS 1 we just get a float.
        oss << msg->recv_timestamp << ";";

        count++;
    }
    oss << "|";

    // NOTE(Jack): We want to have the same semantics as the ROS1 case where if there are no messages in the topic we
    // return std::nullopt. But ROS2 does not provide us a way to query this directly so we have to count manually in
    // the loop and then check.
    if (count == 0) {
        return std::nullopt;
    }

    return oss.str();
}

}  // namespace reprojection::ros2
