#include "serialization.hpp"

#include "types/eigen_types.hpp"

namespace reprojection::database {

protobuf_serialization::ArrayX2dProto Serialize(ArrayX2d const& eigen_array) {
    protobuf_serialization::ArrayX2dProto array_x2d_proto;

    // TOOD(Jack): See the note in the protobuf definition file, but this logic is essentially copied exactly from the
    //  target pixel serialization code. If we used a common protobuf definition (which we should!) then we could
    //  eliminate the copy and  paste code/logic here.
    array_x2d_proto.set_rows(static_cast<int>(eigen_array.rows()));
    auto* const proto_data{array_x2d_proto.mutable_array_data()};
    proto_data->Resize(static_cast<int>(eigen_array.size()), 0);

    Eigen::Map<ArrayX2d>(proto_data->mutable_data(), eigen_array.rows(), eigen_array.cols()) = eigen_array;

    return array_x2d_proto;
}

std::optional<ArrayX2d> Deserialize(protobuf_serialization::ArrayX2dProto const& eigen_array_proto) {
    if (not ValidateDimensions(eigen_array_proto.rows(), 2, eigen_array_proto.array_data_size())) {
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    ArrayX2d eigen_array;
    eigen_array.resize(eigen_array_proto.rows(), 2);
    eigen_array = Eigen::Map<ArrayX2d const>(eigen_array_proto.array_data().data(), eigen_array_proto.rows(), 2);

    return eigen_array;
}

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
}  // LCOV_EXCL_LINE

std::optional<ExtractedTarget> Deserialize(protobuf_serialization::ExtractedTargetProto const& extracted_target_proto) {
    auto const& bundle_proto{extracted_target_proto.bundle()};
    if (not ValidateDimensions(bundle_proto.pixel_rows(), 2, bundle_proto.pixel_data_size())) {
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    if (not ValidateDimensions(bundle_proto.point_rows(), 3, bundle_proto.point_data_size())) {
        return std::nullopt;  // LCOV_EXCL_LINE
    }

    if (not ValidateDimensions(extracted_target_proto.indices_rows(), 2, extracted_target_proto.indices_data_size())) {
        return std::nullopt;  // LCOV_EXCL_LINE
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