#pragma once

#include "types/calibration_types.hpp"
#include "types/io.hpp"

namespace reprojection::steps {

// NOTE(Jack): I had originally planned to note store the images in the database because it would require more
// reading/writing than just feeding the images directly into the feature extractor. However, given that the images will
// be cached after the first execution there is really no argument not to do this. But it will lead to the database
// getting large (even though we only store .png encoded blobs). The benefit is that it makes our downstream workflow
// and database visualization extremely consistent.

struct ImageLoading {
    std::string sensor_name_;
    std::string cache_key_;
    ImageSourceSignature image_source_;

    CalibrationStep step_type{CalibrationStep::ImageLoading};

    std::string EntityId() const { return sensor_name_; }

    std::string HashInputs() const;

    std::shared_ptr<EncodedImages> Compute() const;

    std::shared_ptr<EncodedImages> Load(SqlitePtr const db) const;

    void Save(std::shared_ptr<EncodedImages const> const images, SqlitePtr const db) const;
};

}  // namespace reprojection::steps
