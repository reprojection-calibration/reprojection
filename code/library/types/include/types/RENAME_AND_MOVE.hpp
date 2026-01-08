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

    CameraModel camera_model() const { return data_.sensor.camera_model; }

    ArrayXd initial_intrinsics() const { return data_.initial_intrinsics; }

    class Iterator {
       public:
        using MapIter = CalibrationData::iterator;

        explicit Iterator(MapIter it) : it_{it} {}

        // TODO DO WE NEED THIS?
        InitializationFrameView operator*() const { return {it_->second.extracted_target, it_->second.initial_pose}; }

        // TODO DO WE NEED THIS?
        Iterator& operator++() {
            ++it_;
            return *this;
        }

        // TODO DO WE NEED THIS?
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