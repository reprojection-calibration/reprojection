#pragma once

#include <optional>

#include "types/calibration_types.hpp"
#include "types/camera_types.hpp"
#include "types/eigen_types.hpp"

// TODO(Jack): It looks like there will be a lot of copy and pasted code between the views, eliminate this!
// TODO(Jack): In the same way that an optimization frame view cannot exist without an initial pose, a initialization
// frame cannot exist without extracted targets. Introduce the same pattern of "skip valid" here.

namespace reprojection {

class InitializationFrameView {
   public:
    InitializationFrameView(ExtractedTarget const& extracted_target, std::optional<Array6d>& initial_pose)
        : extracted_target_{extracted_target}, initial_pose_{initial_pose} {}

    ExtractedTarget const& extracted_target() const { return extracted_target_; }

    std::optional<Array6d>& initial_pose() { return initial_pose_; }

   private:
    ExtractedTarget const& extracted_target_;
    std::optional<Array6d>& initial_pose_;
};

class InitializationDataView {
   public:
    explicit InitializationDataView(CameraCalibrationData& data) : data_{data} {}

    CameraModel const& camera_model() const { return data_.sensor.camera_model; }

    ImageBounds const& image_bounds() const { return data_.sensor.bounds; }

    ArrayXd const& initial_intrinsics() const { return data_.initial_intrinsics; }

    class Iterator {
       public:
        using DataFrameIterator = CameraFrameSequence::iterator;

        explicit Iterator(DataFrameIterator it) : it_{it} {}

        InitializationFrameView operator*() const { return {it_->second.extracted_target, it_->second.initial_pose}; }

        Iterator& operator++() {
            ++it_;
            return *this;
        }

        bool operator!=(Iterator const& other) const { return it_ != other.it_; }

       private:
        DataFrameIterator it_;
    };

    Iterator begin() { return Iterator{std::begin(data_.frames)}; }

    Iterator end() { return Iterator{std::end(data_.frames)}; }

   private:
    CameraCalibrationData& data_;
};

}  // namespace reprojection