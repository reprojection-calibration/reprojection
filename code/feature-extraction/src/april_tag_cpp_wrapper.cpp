extern "C" {
#include <apriltag/apriltag.h>
}

#include "april_tag_cpp_wrapper.hpp"

namespace reprojection_calibration::feature_extraction {

// WARN(Jack): If the user passes mismatched _tag_family and _tag_family_destroy this class will not do what they
// actually want it to! If for example the user did the following:
//
//      AprilTagFamily(tagCustom36h11_create(), tag25h9_destroy)
//
// Then this is a huge footgun because the created tag family does not match the one that will be destroyed when the
// destructor is called. Because of the nature of the generated code there is no way to enforce that matching pairs
// are passed, or that the proper destructor is called automatically. This is our best attempt given our options.
AprilTagFamily::AprilTagFamily(apriltag_family_t* _tag_family,
                               std::function<void(apriltag_family_t*)> _tag_family_destroy)
    : tag_family{_tag_family}, tag_family_destroy{std::move(_tag_family_destroy)} {}

AprilTagFamily::~AprilTagFamily() { tag_family_destroy(tag_family); }

// ERROR(Jack): Usage of const_cast<double*>
AprilTagDetection::AprilTagDetection(apriltag_detection_t const& raw_detection)
    : id{raw_detection.id},
      H{Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>{raw_detection.H->data}},
      c{Eigen::Vector2d{raw_detection.c[0], raw_detection.c[1]}},
      p{Eigen::Map<Eigen::Matrix<double, 4, 2, Eigen::RowMajor>>{const_cast<double*>(&raw_detection.p[0][0])}} {}

AprilTagDetector::AprilTagDetector(AprilTagFamily const& tag_family, AprilTagDetectorSettings const& settings)
    : tag_detector{apriltag_detector_create()} {
    apriltag_detector_add_family(tag_detector, tag_family.tag_family);

    tag_detector->quad_decimate = settings.decimate;
    tag_detector->quad_sigma = settings.blur;
    tag_detector->nthreads = settings.threads;
    tag_detector->debug = settings.debug;
    tag_detector->refine_edges = settings.refine_edges;
}

// WARN(Jack): Must be grayscale image
std::vector<AprilTagDetection> AprilTagDetector::Detect(cv::Mat const& gray) const {
    image_u8_t raw_gray{gray.cols, gray.rows, gray.cols, gray.data};
    zarray_t* const raw_detections{apriltag_detector_detect(tag_detector, &raw_gray)};

    std::vector<AprilTagDetection> detections;
    for (int i{0}; i < raw_detections->size; i++) {
        apriltag_detection_t* raw_detection;
        zarray_get(raw_detections, i, &raw_detection);
        detections.emplace_back(*raw_detection);
    }

    apriltag_detections_destroy(raw_detections);

    return detections;
}

AprilTagDetector::~AprilTagDetector() { apriltag_detector_destroy(tag_detector); }

}  // namespace reprojection_calibration::feature_extraction
