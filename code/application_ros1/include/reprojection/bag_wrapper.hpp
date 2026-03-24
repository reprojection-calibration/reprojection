#pragma once

#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>

#include <string>

namespace reprojection::ros1 {

struct BagError {
    std::string message;
};

struct BagWrapper {
    std::unique_ptr<rosbag::Bag> bag;

    explicit BagWrapper(std::unique_ptr<rosbag::Bag> b) : bag(std::move(b)) {}

    ~BagWrapper() {
        try {
            bag->close();
        } catch (...) {
            // Avoids throwing in destructor!
        }
    }

    static std::variant<BagWrapper, BagError> open(std::string const& path, rosbag::BagMode mode) {
        try {
            auto bag = std::make_unique<rosbag::Bag>();
            bag->open(path, mode);
            return BagWrapper{std::move(bag)};
        } catch (std::exception const& e) {
            return BagError{e.what()};
        }
    }
};

struct SingleTopicBagReader {
    std::string topic;
    BagWrapper bag;
    rosbag::View view;

    SingleTopicBagReader(std::string t, BagWrapper&& b)
        : topic(std::move(t)), bag(std::move(b)), view(bag.bag, rosbag::TopicQuery({topic})) {}

    static std::variant<SingleTopicBagReader, BagError> create(std::string const& path, std::string const& topic) {
        auto bag_result = BagWrapper::open(path, rosbag::bagmode::Read);

        if (std::holds_alternative<BagError>(bag_result)) {
            return std::get<BagError>(bag_result);
        }

        auto& bag = std::get<BagWrapper>(bag_result);
        return SingleTopicBagReader{topic, std::move(bag)};
    }
};

}  // namespace reprojection::ros1