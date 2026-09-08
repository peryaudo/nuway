// Multi-channel occupancy grid conventions of docs/02_interfaces.md §1/§4:
// GridSpec, world <-> grid index conversion and bilinear sampling of one
// channel. Grids are in base_link; rows index x (forward), columns index y
// (left); cell (i, j) center is (x_min + (i + 0.5) res, y_min + (j + 0.5) res).
// Mirrored by nuway_ml/common/occupancy.py (parity-tested). Introduced in M0.
#ifndef NUWAY_COMMON_OCCUPANCY_H_
#define NUWAY_COMMON_OCCUPANCY_H_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace nuway_common {

// Grid geometry. Defaults are the stack-wide values of docs/02 §1.
struct GridSpec {
  double resolution = 0.5;  // m per cell
  double x_min = -50.0;     // m, base_link
  double y_min = -50.0;     // m, base_link
  int height = 200;         // rows (x)
  int width = 200;          // cols (y)
};

// Fixed channel order of OccupancyGridMC.
enum class OccupancyChannel : std::uint8_t {
  kOccupied = 0,
  kFree = 1,
  kUnknown = 2,
  kDrivable = 3,
  kHeightMax = 4,
  kDynamic = 5,
};
constexpr int kNumOccupancyChannels = 6;
constexpr std::array<std::string_view, kNumOccupancyChannels>
    kOccupancyChannelNames = {"occupied", "free",       "unknown",
                              "drivable", "height_max", "dynamic"};

// Integer cell index.
struct GridIndex {
  int row = 0;
  int col = 0;
};

// Continuous grid coordinates: (row, col) such that integer values fall on
// cell centers.
struct GridCoord {
  double row = 0.0;
  double col = 0.0;
};

// World (base_link) -> continuous grid coordinates (not clamped).
inline GridCoord WorldToGridCoord(const GridSpec& spec, double x, double y) {
  return GridCoord{((x - spec.x_min) / spec.resolution) - 0.5,
                   ((y - spec.y_min) / spec.resolution) - 0.5};
}

// World -> integer cell containing (x, y); nullopt when outside the grid.
inline std::optional<GridIndex> WorldToGrid(const GridSpec& spec, double x,
                                            double y) {
  const int row =
      static_cast<int>(std::floor((x - spec.x_min) / spec.resolution));
  const int col =
      static_cast<int>(std::floor((y - spec.y_min) / spec.resolution));
  if (row < 0 || row >= spec.height || col < 0 || col >= spec.width) {
    return std::nullopt;
  }
  return GridIndex{row, col};
}

// Cell center of (row, col) in world (base_link) coordinates.
inline std::array<double, 2> GridToWorld(const GridSpec& spec, int row,
                                         int col) {
  return {spec.x_min + ((row + 0.5) * spec.resolution),
          spec.y_min + ((col + 0.5) * spec.resolution)};
}

// Bilinear sample of one row-major [height][width] channel at world (x, y).
// Outside the grid (beyond the outermost cell centers) returns `outside`.
template <typename Scalar>
double BilinearSample(const GridSpec& spec, const Scalar* channel, double x,
                      double y, double outside = 0.0) {
  const GridCoord coord = WorldToGridCoord(spec, x, y);
  if (spec.height < 2 || spec.width < 2 || coord.row < 0.0 || coord.col < 0.0 ||
      coord.row > spec.height - 1 || coord.col > spec.width - 1) {
    return outside;
  }
  int r0 = static_cast<int>(std::floor(coord.row));
  int c0 = static_cast<int>(std::floor(coord.col));
  r0 = std::min(r0, spec.height - 2);
  c0 = std::min(c0, spec.width - 2);
  const double fr = coord.row - r0;
  const double fc = coord.col - c0;
  const auto at = [&](int row, int col) {
    return static_cast<double>(channel[(static_cast<std::size_t>(row) *
                                        static_cast<std::size_t>(spec.width)) +
                                       static_cast<std::size_t>(col)]);
  };
  const double top = ((1.0 - fc) * at(r0, c0)) + (fc * at(r0, c0 + 1));
  const double bottom =
      ((1.0 - fc) * at(r0 + 1, c0)) + (fc * at(r0 + 1, c0 + 1));
  return ((1.0 - fr) * top) + (fr * bottom);
}

template <typename Scalar>
double BilinearSample(const GridSpec& spec, const std::vector<Scalar>& channel,
                      double x, double y, double outside = 0.0) {
  return BilinearSample(spec, channel.data(), x, y, outside);
}

}  // namespace nuway_common

#endif  // NUWAY_COMMON_OCCUPANCY_H_
