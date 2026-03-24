#pragma once

#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>

#include <memory>
#include <string>
#include <variant>

namespace reprojection::ros1 {

struct BagError {
    std::string message;
};

struct SingleTopicBagReader {
    std::string topic;
    std::unique_ptr<rosbag::Bag> bag;
    std::unique_ptr<rosbag::View> view;

    static std::variant<SingleTopicBagReader, BagError> Create(std::string const& path, std::string const& topic) {
        try {
            auto bag{std::make_unique<rosbag::Bag>()};
            bag->open(path, rosbag::bagmode::Read);

            return SingleTopicBagReader{topic, std::move(bag)};
        } catch (std::exception const& e) {
            return BagError{e.what()};
        }
    }

   private:
    SingleTopicBagReader(std::string _topic, std::unique_ptr<rosbag::Bag> _bag)
        : topic(std::move(_topic)),
          bag(std::move(_bag)),
          view(std::make_unique<rosbag::View>(*bag, rosbag::TopicQuery({topic}))) {}
};

}  // namespace reprojection::ros1