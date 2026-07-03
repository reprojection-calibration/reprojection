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
    std::string topic_;
    std::unique_ptr<rosbag::Bag> bag_;
    std::unique_ptr<rosbag::View> view_;

    static std::variant<SingleTopicBagReader, BagError> Create(std::string const& path, std::string const& topic) {
        try {
            auto bag{std::make_unique<rosbag::Bag>()};
            bag->open(path, rosbag::bagmode::Read);

            return SingleTopicBagReader{topic, std::move(bag)};
        } catch (...) {
            // TODO(Jack): Instead of just swallowing the exception here we should use it to give the user some more
            // useful output.
            return BagError{"Error loading data: " + path};
        }
    }

    // Rule of five requirement because we define a destructor.
    SingleTopicBagReader(SingleTopicBagReader&&) = default;
    SingleTopicBagReader& operator=(SingleTopicBagReader&&) = default;
    SingleTopicBagReader(SingleTopicBagReader const&) = delete;
    SingleTopicBagReader& operator=(SingleTopicBagReader const&) = delete;

    ~SingleTopicBagReader() {
        if (bag_ != nullptr) {
            // TODO(Jack): Can this throw? Do we need to worry about that?
            bag_->close();
        }
    }

   private:
    // TODO(Jack): To be perfectly honest I am not 100% sure here about the RAII semantics/rule of 0/3/5, but the code
    //  works and does not segfault. When it does we can look closer :)
    SingleTopicBagReader(std::string_view topic, std::unique_ptr<rosbag::Bag> bag)
        : topic_{topic},
          bag_(std::move(bag)),
          view_(std::make_unique<rosbag::View>(*bag_, rosbag::TopicQuery({topic_}))) {}
};

}  // namespace reprojection::ros1