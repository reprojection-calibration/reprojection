#include "serialization.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

#include "types/algorithm_types.hpp"

using namespace reprojection;

TEST(DatabaseSerialization, TestArrayX2dSerialization) {
    ArrayX2d const original{{1.23, 1.43}, {2.75, 2.35}, {200.24, 300.56}};

    protobuf_serialization::ArrayX2dProto const serialized{database::Serialize(original)};
    auto const deserialized_opt{database::Deserialize(serialized)};

    ASSERT_TRUE(deserialized_opt.has_value());
    ArrayX2d const array{deserialized_opt.value()};

    EXPECT_TRUE(array.isApprox(original));
}

TEST(DatabaseSerialization, TestArrayX2dSerializationEmpty) {
    ArrayX2d const original;
    protobuf_serialization::ArrayX2dProto const serialized{database::Serialize(original)};

    EXPECT_EQ(serialized.array_data_size(), 0);
}

TEST(DatabaseSerialization, TestArrayX2dDeserializationEmpty) {
    protobuf_serialization::ArrayX2dProto const original;
    auto const deserialized_opt{database::Deserialize(original)};

    ASSERT_TRUE(deserialized_opt.has_value());
    EXPECT_EQ(deserialized_opt->size(), 0);
}

TEST(DatabaseSerialization, TestExtractedTargetSerialization) {
    ExtractedTarget const original{Bundle{MatrixX2d{{1.23, 1.43}, {2.75, 2.35}, {200.24, 300.56}},
                                          MatrixX3d{{3.25, 3.45, 5.43}, {6.18, 6.78, 4.56}, {300.65, 200.56, 712.57}}},
                                   {{5, 6}, {2, 3}, {650, 600}}};

    protobuf_serialization::ExtractedTargetProto const serialized{database::Serialize(original)};
    auto const deserialized_opt{database::Deserialize(serialized)};

    ASSERT_TRUE(deserialized_opt.has_value());
    auto const [bundle, indices]{deserialized_opt.value()};

    EXPECT_TRUE(bundle.pixels.isApprox(original.bundle.pixels));
    EXPECT_TRUE(bundle.points.isApprox(original.bundle.points));
    EXPECT_TRUE(indices.isApprox(original.indices));
}

TEST(DatabaseSerialization, TestExtractedTargetSerializationEmpty) {
    ExtractedTarget const original;
    protobuf_serialization::ExtractedTargetProto const serialized{database::Serialize(original)};

    EXPECT_TRUE(serialized.has_bundle());
    EXPECT_EQ(serialized.bundle().pixel_data_size(), 0);
    EXPECT_EQ(serialized.bundle().point_data_size(), 0);
    EXPECT_EQ(serialized.indices_data_size(), 0);
}

TEST(DatabaseSerialization, TestExtractedTargetDeserializationEmpty) {
    protobuf_serialization::ExtractedTargetProto const original;
    auto const deserialized_opt{database::Deserialize(original)};

    ASSERT_TRUE(deserialized_opt.has_value());
    EXPECT_EQ(deserialized_opt->bundle.pixels.size(), 0);
    EXPECT_EQ(deserialized_opt->bundle.points.size(), 0);
    EXPECT_EQ(deserialized_opt->indices.size(), 0);
}
