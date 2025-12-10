#include "serialization.hpp"

#include "types/eigen_types.hpp"

namespace reprojection::database {

protobuf_serialization::ExtractedTargetProto Serialize(ExtractedTarget const& target) {
    protobuf_serialization::ExtractedTargetProto extracted_target_proto;
    protobuf_serialization::BundleProto* const bundle_proto{extracted_target_proto.mutable_bundle()};

    // Pixels
    auto const& pixels{target.bundle.pixels};

    bundle_proto->set_pixel_rows(static_cast<int>(pixels.rows()));
    auto* const pixel_data{bundle_proto->mutable_pixel_data()};
    pixel_data->Resize(static_cast<int>(pixels.size()), 0.0);

    Eigen::Map<MatrixX2d>(pixel_data->mutable_data(), pixels.rows(), pixels.cols()) = pixels;

    // Points
    auto const& points{target.bundle.points};

    bundle_proto->set_point_rows(static_cast<int>(points.rows()));
    auto* const point_data{bundle_proto->mutable_point_data()};
    point_data->Resize(static_cast<int>(points.size()), 0.0);

    Eigen::Map<MatrixX3d>(point_data->mutable_data(), points.rows(), points.cols()) = points;

    // Indices
    auto const& indices{target.indices};

    extracted_target_proto.set_indices_rows(static_cast<int>(indices.rows()));
    auto* const indices_data{extracted_target_proto.mutable_indices_data()};
    indices_data->Resize(static_cast<int>(indices.size()), 0);

    Eigen::Map<ArrayX2i>(indices_data->mutable_data(), indices.rows(), indices.cols()) = indices;

    return extracted_target_proto;
}

std::optional<ExtractedTarget> Deserialize(protobuf_serialization::ExtractedTargetProto const& extracted_target_proto) {
    auto const& bundle_proto{extracted_target_proto.bundle()};
    if (not ValidateDimensions(bundle_proto.pixel_rows(), 2, bundle_proto.pixel_data_size())) {
        return std::nullopt;
    }

    if (not ValidateDimensions(bundle_proto.point_rows(), 3, bundle_proto.point_data_size())) {
        return std::nullopt;
    }

    if (not ValidateDimensions(extracted_target_proto.indices_rows(), 2, extracted_target_proto.indices_data_size())) {
        return std::nullopt;
    }

    ExtractedTarget target;
    target.bundle.pixels.resize(bundle_proto.pixel_rows(), 2);
    target.bundle.pixels = Eigen::Map<MatrixX2d const>(bundle_proto.pixel_data().data(), bundle_proto.pixel_rows(), 2);

    target.bundle.points.resize(bundle_proto.point_rows(), 3);
    target.bundle.points = Eigen::Map<MatrixX3d const>(bundle_proto.point_data().data(), bundle_proto.point_rows(), 3);

    target.indices.resize(extracted_target_proto.indices_rows(), 2);
    target.indices = Eigen::Map<ArrayX2i const>(extracted_target_proto.indices_data().data(),
                                                extracted_target_proto.indices_rows(), 2);

    return target;
}

}  // namespace reprojection::database