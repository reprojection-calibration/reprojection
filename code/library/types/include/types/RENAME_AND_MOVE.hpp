#pragma once

#include <map>

#include "types/calibration_types.hpp"
#include "types/eigen_types.hpp"

namespace reprojection {

// for every camera pose, which is unique according to timestamp and sensor name, we will have a image, extracted
// target, initial pose, optimized pose, initial reprojection error and reprojection error after optimization

// TODO(Jack): Proper naming for all these types!!!!
struct CameraSensor {
    std::string sensor_name;
    CameraModel camera_model;
};

struct CalibrationFrame {
    ExtractedTarget extracted_target;
    Vector6d initial_pose;
    Vector6d optimized_pose;
};

using CalibrationData = std::map<std::uint64_t, CalibrationFrame>;

struct CameraSensorData {
    CameraSensor sensor;
    ArrayXd initial_intrinsics;
    ArrayXd optimized_intrinsics;

    CalibrationData frames;
};

class InitializationFrameView {
   public:
    InitializationFrameView(ExtractedTarget const& extracted_target, Vector6d& initial_pose)
        : extracted_target_{extracted_target}, initial_pose_{initial_pose} {}

    // TODO(Jack): Does the const in the return here really do what I think/want it to do? That is to protect the
    // extracted target from modification?
    ExtractedTarget const& extracted_target() const { return extracted_target_; }

    Vector6d& initial_pose() { return initial_pose_; }

   private:
    ExtractedTarget const& extracted_target_;
    Vector6d& initial_pose_;
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

// TODO(Jack): This is way too generic of a name! This specifically is the optimization of camera poses and intrinsics
// after the linear initialization.
class OptimizationFrameView {
   public:
    OptimizationFrameView(ExtractedTarget const& extracted_target, Vector6d const& initial_pose,
                          Vector6d& optimized_pose)
        : extracted_target_{extracted_target}, initial_pose_{initial_pose}, optimized_pose_{optimized_pose} {}

    // TODO(Jack): Does the const in the return here really do what I think/want it to do? That is to protect the
    // extracted target from modification?
    ExtractedTarget const& extracted_target() const { return extracted_target_; }

    Vector6d const& initial_pose() const { return initial_pose_; }

    Vector6d& optimized_pose() { return optimized_pose_; }

   private:
    ExtractedTarget const& extracted_target_;
    Vector6d const& initial_pose_;
    Vector6d& optimized_pose_;
};

class OptimizationDataView {
   public:
    explicit OptimizationDataView(CameraSensorData& data) : data_{data} {}

    CameraModel const& camera_model() const { return data_.sensor.camera_model; }

    ArrayXd const& initial_intrinsics() const { return data_.initial_intrinsics; }

    ArrayXd& optimized_intrinsics() { return data_.optimized_intrinsics; }

    class Iterator {
       public:
        using MapIter = CalibrationData::iterator;

        explicit Iterator(MapIter it) : it_{it} {}

        OptimizationFrameView operator*() const {
            return {it_->second.extracted_target, it_->second.initial_pose, it_->second.optimized_pose};
        }

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