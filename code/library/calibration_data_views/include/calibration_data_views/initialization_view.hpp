#pragma once

#include "calibration_data_views/view_types.hpp"
#include "types/eigen_types.hpp"

// TODO(Jack): It looks like there will be a lot of copy and pasted code between the views, eliminate this!

namespace reprojection {

class InitializationFrameView {
   public:
    InitializationFrameView(ExtractedTarget const& extracted_target, Array6d& initial_pose)
        : extracted_target_{extracted_target}, initial_pose_{initial_pose} {}

    // TODO(Jack): Does the const in the return here really do what I think/want it to do? That is to protect the
    // extracted target from modification?
    ExtractedTarget const& extracted_target() const { return extracted_target_; }

    Array6d& initial_pose() { return initial_pose_; }

   private:
    ExtractedTarget const& extracted_target_;
    Array6d& initial_pose_;
};

class InitializationDataView {
   public:
    explicit InitializationDataView(CameraSensorData& data) : data_{data} {}

    CameraModel const& camera_model() const { return data_.sensor.camera_model; }

    ArrayXd const& initial_intrinsics() const { return data_.initial_intrinsics; }

    class Iterator {
       public:
        using MapIter = CalibrationData::iterator;

        explicit Iterator(MapIter it) : it_{it} {}

        InitializationFrameView operator*() const { return {it_->second.extracted_target, it_->second.initial_pose}; }

        Iterator& operator++() {
            ++it_;
            return *this;
        }

        bool operator!=(Iterator const& other) const { return it_ != other.it_; }

       private:
        MapIter it_;
    };

    Iterator begin() { return Iterator{std::begin(data_.frames)}; }

    Iterator end() { return Iterator{std::end(data_.frames)}; }

   private:
    CameraSensorData& data_;
};

}  // namespace reprojection