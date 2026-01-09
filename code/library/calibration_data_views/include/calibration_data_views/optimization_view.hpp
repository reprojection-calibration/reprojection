#pragma once

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection {

// TODO(Jack): This is way too generic of a name! This specifically is the optimization of camera poses and intrinsics
// after the linear initialization.
class OptimizationFrameView {
   public:
    OptimizationFrameView(uint64_t const& timestamp_ns, ExtractedTarget const& extracted_target,
                          Array6d const& initial_pose, Array6d& optimized_pose)
        : timestamp_ns_{timestamp_ns},
          extracted_target_{extracted_target},
          initial_pose_{initial_pose},
          optimized_pose_{optimized_pose} {}

    // NOTE(Jack): I am actually not thrilled here about adding the timestamp here because it is not actually used as
    // data in the optimization itself. It is used to help track the correspondence of the residual ids (see the
    // optimization function itself). If it is possible to easily achieve this same correspondence without using the
    // timestamps then we can and I think we should remove the timestamp here.
    uint64_t const& timestamp_ns() const { return timestamp_ns_; }

    ExtractedTarget const& extracted_target() const { return extracted_target_; }

    Array6d const& initial_pose() const { return initial_pose_; }

    Array6d& optimized_pose() { return optimized_pose_; }

   private:
    uint64_t const& timestamp_ns_;
    ExtractedTarget const& extracted_target_;
    Array6d const& initial_pose_;
    Array6d& optimized_pose_;
};

class OptimizationDataView {
   public:
    explicit OptimizationDataView(CameraCalibrationData& data) : data_{data} {}

    CameraModel const& camera_model() const { return data_.sensor.camera_model; }

    ArrayXd const& initial_intrinsics() const { return data_.initial_intrinsics; }

    ArrayXd& optimized_intrinsics() { return data_.optimized_intrinsics; }

    class Iterator {
       public:
        using DataFrameIterator = CameraFrameSequence::iterator;

        explicit Iterator(DataFrameIterator it) : it_{it} {}

        OptimizationFrameView operator*() const {
            return {it_->first, it_->second.extracted_target, it_->second.initial_pose, it_->second.optimized_pose};
        }

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