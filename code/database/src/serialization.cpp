#include "serialization.hpp"

#include <algorithm>
#include <vector>

namespace reprojection::database {

namespace {

[[nodiscard]] bool validate_dimensions(int rows, int cols, size_t data_size) {
    return (static_cast<size_t>(rows) * static_cast<size_t>(cols)) == data_size;
}

}  // namespace

protobuf_serialization::ExtractedTargetProto Serialize( ExtractedTarget const& target) {
    protobuf_serialization::ExtractedTargetProto proto_out;
    protobuf_serialization::BundleProto* bundle_proto = proto_out.mutable_bundle();

    // 1. Serialize Bundle::pixels (MatrixX2d)
    const auto& pixels = target.bundle.pixels;
    bundle_proto->set_pixel_rows(static_cast<int>(pixels.rows()));

    // Resize protobuf repeated field to fit data
    auto* pixel_data = bundle_proto->mutable_pixel_data();
    pixel_data->Resize(static_cast<int>(pixels.size()), 0.0);

    // Map the protobuf memory and copy from Eigen (Safe copy handling alignment)
    Eigen::Map<Eigen::MatrixX2d>(pixel_data->mutable_data(), pixels.rows(), pixels.cols()) = pixels;

    // 2. Serialize Bundle::points (MatrixX3d)
    const auto& points = target.bundle.points;
    bundle_proto->set_point_rows(static_cast<int>(points.rows()));

    auto* point_data = bundle_proto->mutable_point_data();
    point_data->Resize(static_cast<int>(points.size()), 0.0);

    Eigen::Map<Eigen::MatrixX3d>(point_data->mutable_data(), points.rows(), points.cols()) = points;

    // 3. Serialize ExtractedTarget::indices (ArrayX2i)
    const auto& indices = target.indices;
    proto_out.set_indices_rows(static_cast<int>(indices.rows()));

    auto* indices_data = proto_out.mutable_indices_data();
    indices_data->Resize(static_cast<int>(indices.size()), 0);

    Eigen::Map<Eigen::ArrayX2i>(indices_data->mutable_data(), indices.rows(), indices.cols()) = indices;

    return proto_out;
}

std::optional<ExtractedTarget> Deserialize( protobuf_serialization::ExtractedTargetProto const& proto) {
    // 1. Validate Bundle::pixels
    const auto& bundle_proto = proto.bundle();
    if (!validate_dimensions(bundle_proto.pixel_rows(), 2, bundle_proto.pixel_data_size())) {
        return std::nullopt;
    }

    // 2. Validate Bundle::points
    if (!validate_dimensions(bundle_proto.point_rows(), 3, bundle_proto.point_data_size())) {
        return std::nullopt;
    }

    // 3. Validate ExtractedTarget::indices
    if (!validate_dimensions(proto.indices_rows(), 2, proto.indices_data_size())) {
        return std::nullopt;
    }

    // Construction (RVO optimized)
    ExtractedTarget target;

    // Deserialize Pixels
    target.bundle.pixels.resize(bundle_proto.pixel_rows(), 2);
    target.bundle.pixels =
        Eigen::Map<const Eigen::MatrixX2d>(bundle_proto.pixel_data().data(), bundle_proto.pixel_rows(), 2);

    // Deserialize Points
    target.bundle.points.resize(bundle_proto.point_rows(), 3);
    target.bundle.points =
        Eigen::Map<const Eigen::MatrixX3d>(bundle_proto.point_data().data(), bundle_proto.point_rows(), 3);

    // Deserialize Indices
    target.indices.resize(proto.indices_rows(), 2);
    target.indices = Eigen::Map<const Eigen::ArrayX2i>(proto.indices_data().data(), proto.indices_rows(), 2);

    return target;
}

}  // namespace reprojection::database