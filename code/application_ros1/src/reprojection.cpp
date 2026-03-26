#include "reprojection/reprojection.hpp"

#include <filesystem>

namespace reprojection::ros1 {

std::optional<std::string> SerializeBagTopic(SingleTopicBagReader const& data) {
    if (data.view->size() == 0) {
        return std::nullopt;
    }

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);

    // NOTE(Jack): Underlying our use of this function is the assumption that the bag name, topic name, and the
    // collection of all message timestamps is sufficient to uniquely identify a data stream. Sounds logical right? I
    // think so.
    oss << std::filesystem::path(data.bag->getFileName()).filename().c_str() << "|";
    oss << data.topic << "|";

    for (auto const& msg : *data.view) {
        oss << msg.getTime().toSec() << ";";
    }
    oss << "|";

    return oss.str();
}

}  // namespace reprojection::ros1
