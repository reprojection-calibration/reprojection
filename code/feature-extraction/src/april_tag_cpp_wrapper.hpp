#pragma once

extern "C" {
#include <apriltag/apriltag.h>
}

#include <Eigen/Dense>
#include <functional>
#include <opencv2/opencv.hpp>

// This is my attempt to RAII-ify the  C code from the apriltag repository. The main thing I try to fight here is
// manually having to deallocate memory. For the detector and tag detections themselves that is relatively easy because
// they have generic creation and destruction functions. For the tag family it is trickier because the generated code is
// actually specific to each one (see comment in AprilTagFamily).
//
// WARN(Jack): The const correctness and memory safety of all apriltag related code is not clear at this point
// (26.09.2025)! I am 99% sure that there are some big footguns in here, and we will find some "presents" later.

namespace reprojection_calibration::feature_extraction {

struct AprilTagFamily {
    // WARN(Jack): Footgun, see comment in .cpp
    AprilTagFamily(apriltag_family_t* _tag_family, std::function<void(apriltag_family_t*)> _tag_family_destroy);

    ~AprilTagFamily();

    apriltag_family_t* tag_family;

   private:
    std::function<void(apriltag_family_t*)> tag_family_destroy;
};

struct AprilTagDetection {
    AprilTagDetection() = default;

    explicit AprilTagDetection(apriltag_detection_t const& raw_detection);

    int id{};
    Eigen::Matrix3d H;
    Eigen::Vector2d c;
    Eigen::Matrix<double, 4, 2> p;
};

struct AprilTagDetector {
    struct AprilTagDetectorSettings {
        double decimate;
        double blur;
        int threads;
        bool debug;
        bool refine_edges;
    };

    AprilTagDetector(AprilTagFamily const& tag_family, AprilTagDetectorSettings const& settings);

    // WARN(Jack): Must be grayscale image
    std::vector<AprilTagDetection> Detect(cv::Mat const& gray) const;

    ~AprilTagDetector();

   private:
    apriltag_detector_t* tag_detector;
};

}  // namespace reprojection_calibration::feature_extraction
