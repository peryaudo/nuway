#include "nuway_common/occupancy.hpp"

#include <vector>

#include <gtest/gtest.h>

namespace nuway_common {
namespace {

TEST(OccupancyTest, WorldToGridRoundTripsThroughCellCenters) {
  const GridSpec spec;
  const GridIndex idx =
      WorldToGrid(spec, 0.1, -0.1).value_or(GridIndex{-1, -1});
  EXPECT_EQ(idx.row, 100);
  EXPECT_EQ(idx.col, 99);
  const std::array<double, 2> center = GridToWorld(spec, idx.row, idx.col);
  EXPECT_NEAR(center[0], 0.25, 1e-9);
  EXPECT_NEAR(center[1], -0.25, 1e-9);
  EXPECT_FALSE(WorldToGrid(spec, 50.0, 0.0).has_value());
  EXPECT_FALSE(WorldToGrid(spec, -50.1, 0.0).has_value());
  EXPECT_TRUE(WorldToGrid(spec, -50.0, 49.99).has_value());
}

TEST(OccupancyTest, BilinearSampleInterpolatesBetweenCenters) {
  GridSpec spec;
  spec.resolution = 1.0;
  spec.x_min = 0.0;
  spec.y_min = 0.0;
  spec.height = 2;
  spec.width = 2;
  // channel[row][col]: rows index x.
  const std::vector<float> channel = {0.0F, 1.0F, 2.0F, 3.0F};
  // Cell centers at 0.5 and 1.5. Midpoint (1.0, 1.0) averages all four.
  EXPECT_NEAR(BilinearSample(spec, channel, 1.0, 1.0), 1.5, 1e-9);
  EXPECT_NEAR(BilinearSample(spec, channel, 0.5, 0.5), 0.0, 1e-9);
  EXPECT_NEAR(BilinearSample(spec, channel, 1.5, 0.5), 2.0, 1e-9);
  EXPECT_NEAR(BilinearSample(spec, channel, 0.5, 1.5), 1.0, 1e-9);
  EXPECT_NEAR(BilinearSample(spec, channel, 1.0, 0.5), 1.0, 1e-9);
  // Outside the outermost centers returns `outside`.
  EXPECT_NEAR(BilinearSample(spec, channel, 0.25, 0.5, -1.0), -1.0, 1e-9);
}

TEST(OccupancyTest, ChannelNamesFollowTheFixedOrder) {
  EXPECT_EQ(
      kOccupancyChannelNames[static_cast<int>(OccupancyChannel::kUnknown)],
      "unknown");
  EXPECT_EQ(
      kOccupancyChannelNames[static_cast<int>(OccupancyChannel::kDynamic)],
      "dynamic");
}

}  // namespace
}  // namespace nuway_common
