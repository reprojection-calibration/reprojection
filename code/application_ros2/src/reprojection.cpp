#include "application_ros2/reprojection.hpp"

#include <filesystem>
#include <iostream>

#include "msg_parsing.hpp"

namespace reprojection::ros2 {

std::optional<std::string> SerializeBagTopic(SingleTopicBagReader const& data) {
    std::ostringstream oss;
    oss << std::filesystem::path(data.reader_->get_metadata().files[0].path).filename().c_str() << "|";
    oss << data.topic_ << "|";

    int count{0};
    while (auto const msg{data.Next()}) {
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

ImageSource::ImageSource(SingleTopicBagReader& bag_reader) : bag_reader_{bag_reader} {}

std::optional<std::pair<uint64_t, cv::Mat>> ImageSource::operator()() {
    if (auto const msg{bag_reader_.Next()}) {
        return ros2::ToCvMat(*msg, bag_reader_.topic_type_);
    }

    return std::nullopt;
}

ImuSource::ImuSource(SingleTopicBagReader& bag_reader) : bag_reader_{bag_reader} {}

std::optional<std::pair<uint64_t, std::array<double, 6>>> ImuSource::operator()() {
    if (auto const msg{bag_reader_.Next()}) {
        return ros2::ToImuArray(*msg);
    }

    return std::nullopt;
}

}  // namespace reprojection::ros2
