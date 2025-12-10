#include "serialization.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "types/calibration_types.hpp"

using namespace reprojection;

TEST(DatabaseSerialization, TestSerialization) {
    ExtractedTarget const original{Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}}, MatrixX3d{{3.25, 3.45}, {6.18, 6.78}}},
                                   {{5, 6}, {2, 3}}};

    protobuf_serialization::ExtractedTargetProto const serialized{database::Serialize(original)};
    auto const deserialized_opt{database::Deserialize(serialized)};

    ASSERT_TRUE(deserialized_opt.has_value());
    ExtractedTarget const deserialized{deserialized_opt.value()};

    EXPECT_TRUE(deserialized.bundle.pixels.isApprox(original.bundle.pixels));
    EXPECT_TRUE(deserialized.bundle.points.isApprox(original.bundle.points));
    EXPECT_TRUE(deserialized.indices.isApprox(original.indices));
}