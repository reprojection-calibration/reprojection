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

    BagWrapper(BagWrapper&&) noexcept = default;
    BagWrapper& operator=(BagWrapper&&) noexcept = default;

    BagWrapper(BagWrapper const&) = delete;
    BagWrapper& operator=(BagWrapper const&) = delete;

    // ERROR(Jack): Where does bag.close() get called? We are leaking memory!

    static std::variant<BagWrapper, BagError> Open(std::string const& path, rosbag::BagMode const mode) {
        try {
            auto bag{std::make_unique<rosbag::Bag>()};
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
    std::unique_ptr<rosbag::View> view;

    SingleTopicBagReader(std::string t, BagWrapper&& b)
        : topic(std::move(t)),
          bag(std::move(b)),
          view(std::make_unique<rosbag::View>(*bag.bag, rosbag::TopicQuery({topic}))) {}

    static std::variant<SingleTopicBagReader, BagError> Create(std::string const& path, std::string const& topic) {
        auto bag_result = BagWrapper::Open(path, rosbag::bagmode::Read);

        if (std::holds_alternative<BagError>(bag_result)) {
            return std::get<BagError>(bag_result);
        }

        auto& bag = std::get<BagWrapper>(bag_result);

        return SingleTopicBagReader{topic, std::move(bag)};
    }
};

}  // namespace reprojection::ros1