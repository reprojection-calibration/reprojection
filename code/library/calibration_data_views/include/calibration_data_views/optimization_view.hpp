#pragma once

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection {

// TODO(Jack): This is way too generic of a name! This specifically is the optimization of camera poses and intrinsics
// after the linear initialization.
class OptimizationFrameView {
   public:
    OptimizationFrameView(uint64_t const& timestamp_ns, ExtractedTarget const& extracted_target,
                          Array6d const& initial_pose, std::optional<ArrayX2d>& initial_reprojection_error,
                          std::optional<Array6d>& optimized_pose, std::optional<ArrayX2d>& optimized_reprojection_error)
        : timestamp_ns_{timestamp_ns},
          extracted_target_{extracted_target},
          initial_pose_{initial_pose},
          initial_reprojection_error_{initial_reprojection_error},
          optimized_pose_{optimized_pose},
          optimized_reprojection_error_{optimized_reprojection_error} {}

    uint64_t const& timestamp_ns() const { return timestamp_ns_; }

    ExtractedTarget const& extracted_target() const { return extracted_target_; }

    Array6d const& initial_pose() const { return initial_pose_; }

    std::optional<ArrayX2d>& initial_reprojection_error() { return initial_reprojection_error_; }

    std::optional<Array6d>& optimized_pose() { return optimized_pose_; }

    std::optional<ArrayX2d>& optimized_reprojection_error() { return optimized_reprojection_error_; }

   private:
    uint64_t const& timestamp_ns_;
    ExtractedTarget const& extracted_target_;
    Array6d const& initial_pose_;
    std::optional<ArrayX2d>& initial_reprojection_error_;
    std::optional<Array6d>& optimized_pose_;
    std::optional<ArrayX2d>& optimized_reprojection_error_;
};

class OptimizationDataView {
   public:
    // TODO(Jack): What is happening here in the constructor does not have so much to do with being a view as about
    // intializing the data structure for optimization. Does this belong here or somewhere else?
    explicit OptimizationDataView(CameraCalibrationData& data) : data_{data} {
        data_.optimized_intrinsics = data_.initial_intrinsics;

        for (auto& [_, frame_i] : data_.frames) {
            frame_i.optimized_pose = frame_i.initial_pose;
        }
    }

    CameraModel const& camera_model() const { return data_.sensor.camera_model; }

    ImageBounds const& image_bounds() const { return data_.sensor.bounds; }

    ArrayXd const& initial_intrinsics() const { return data_.initial_intrinsics; }

    ArrayXd& optimized_intrinsics() { return data_.optimized_intrinsics; }

    class Iterator {
       public:
        // To understand iterator traits - https://en.cppreference.com/w/cpp/iterator/iterator_traits.html
        using iterator_category = std::forward_iterator_tag;
        using value_type = OptimizationFrameView;
        using difference_type = std::ptrdiff_t;
        using pointer = void;
        using reference = value_type;

        using DataFrameIterator = CameraFrameSequence::iterator;

        explicit Iterator(DataFrameIterator it, DataFrameIterator end) : it_{it}, end_{end} { SkipInvalid(); }

        // NOTE(Jack): We have an unprotected optional dereference here because we only consider frames with an initial
        // pose as valid (see the SkipInvalid() method). If however we mess this implementation up somehow it might be
        // that we get segfaults here due to invalid optional dereference.
        OptimizationFrameView operator*() const {
            return {it_->first,
                    it_->second.extracted_target,
                    it_->second.initial_pose.value(),
                    it_->second.initial_reprojection_error,
                    it_->second.optimized_pose,
                    it_->second.optimized_reprojection_error};
        }

        Iterator& operator++() {
            ++it_;
            SkipInvalid();

            return *this;
        }

        bool operator==(Iterator const& other) const { return it_ == other.it_; }

        bool operator!=(Iterator const& other) const { return not(*this == other); }

       private:
        void SkipInvalid() {
            // WARN(Jack): In the condition we need to check the end condition first otherwise we will get undefined
            // behavior when dereferencing before checking if we are at the end.
            while (it_ != end_ and not it_->second.initial_pose.has_value()) {
                ++it_;
            }
        }

        DataFrameIterator it_;
        DataFrameIterator end_;
    };

    Iterator begin() { return Iterator{std::begin(data_.frames), std::end(data_.frames)}; }

    Iterator cbegin() const { return Iterator{std::begin(data_.frames), std::end(data_.frames)}; }

    Iterator end() { return Iterator{std::end(data_.frames), std::end(data_.frames)}; }

    Iterator cend() const { return Iterator{std::end(data_.frames), std::end(data_.frames)}; }

    std::size_t valid_frame_count() const { return std::distance(cbegin(), cend()); }

   private:
    CameraCalibrationData& data_;
};

}  // namespace reprojection