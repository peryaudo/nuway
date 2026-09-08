#include "nuway_control/longitudinal_map.h"

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "nuway_control/vehicle_model.h"

namespace nuway_control {
namespace {

constexpr const char* kYaml = R"(
name: toy
wheelbase: 2.5
max_steer_angle: 1.0
limits: {a_max: 2.5, a_min: -5.0, jerk_max: 5.0, steer_rate_max: 0.5, kappa_max: 0.2}
wheelbase_fitted: 2.6
understeer_gradient: 0.001
longitudinal_map:
  v_bins: [0.0, 10.0, 20.0]
  throttle_bins: [0.0, 0.5, 1.0]
  accel_table:
  - [-0.5, 1.0, 3.0]
  - [-1.0, 0.5, 2.0]
  - [-2.0, 0.0, 1.0]
  brake_bins: [0.0, 0.5, 1.0]
  decel_table:
  - [-0.5, -3.0, -6.0]
  - [-1.0, -4.0, -8.0]
  - [-2.0, -5.0, -9.0]
  coast_accel: [-0.5, -1.0, -2.0]
)";

LongitudinalMap Unwrap(const std::optional<LongitudinalMap>& maybe,
                       const std::string& error) {
  EXPECT_TRUE(maybe.has_value()) << error;
  return maybe.value_or(LongitudinalMap{});
}

TEST(LongitudinalMap, Helpers) {
  const std::vector<double> bins{0.0, 1.0, 2.0};
  const std::vector<double> values{0.0, 10.0, 15.0};
  EXPECT_DOUBLE_EQ(Interp1d(bins, values, 0.5), 5.0);
  EXPECT_DOUBLE_EQ(Interp1d(bins, values, 1.5), 12.5);
  EXPECT_DOUBLE_EQ(Interp1d(bins, values, -3.0), 0.0);
  EXPECT_DOUBLE_EQ(Interp1d(bins, values, 9.0), 15.0);
  EXPECT_DOUBLE_EQ(Interp1d(bins, values, 2.0), 15.0);
  EXPECT_DOUBLE_EQ(InvertMonotone(bins, values, 5.0), 0.5);
  EXPECT_DOUBLE_EQ(InvertMonotone(bins, values, 12.5), 1.5);
  EXPECT_DOUBLE_EQ(InvertMonotone(bins, values, -1.0), 0.0);
  EXPECT_DOUBLE_EQ(InvertMonotone(bins, values, 99.0), 2.0);
  const std::vector<double> decreasing{0.0, -10.0, -15.0};
  EXPECT_DOUBLE_EQ(InvertMonotone(bins, decreasing, -5.0), 0.5);
  EXPECT_DOUBLE_EQ(InvertMonotone(bins, decreasing, 1.0), 0.0);
  EXPECT_DOUBLE_EQ(InvertMonotone(bins, decreasing, -99.0), 2.0);
  const Table table{{0.0, 1.0}, {2.0, 3.0}, {4.0, 5.0}};
  const std::vector<double> row = InterpRow(bins, table, 0.25);
  EXPECT_DOUBLE_EQ(row[0], 0.5);
  EXPECT_DOUBLE_EQ(row[1], 1.5);
  EXPECT_DOUBLE_EQ(InterpRow(bins, table, 7.0)[1], 5.0);
  EXPECT_DOUBLE_EQ(InterpRow(bins, table, 2.0)[0], 4.0);
}

TEST(LongitudinalMap, ForwardAndInverse) {
  std::string error;
  const LongitudinalMap map =
      Unwrap(LongitudinalMap::FromYamlString(kYaml, &error), error);
  EXPECT_DOUBLE_EQ(map.Accel(0.0, 1.0), 3.0);
  EXPECT_DOUBLE_EQ(map.Accel(5.0, 0.5), 0.75);
  EXPECT_DOUBLE_EQ(map.Accel(50.0, 0.0), -2.0);
  EXPECT_DOUBLE_EQ(map.Decel(10.0, 0.25), -2.5);
  EXPECT_DOUBLE_EQ(map.Coast(15.0), -1.5);
  // Throttle branch at and above coast, brake branch below.
  PedalCommand cmd = map.Inverse(10.0, 1.25);
  EXPECT_DOUBLE_EQ(cmd.throttle, 0.75);
  EXPECT_DOUBLE_EQ(cmd.brake, 0.0);
  cmd = map.Inverse(10.0, -1.0);
  EXPECT_DOUBLE_EQ(cmd.throttle, 0.0);
  EXPECT_DOUBLE_EQ(cmd.brake, 0.0);
  cmd = map.Inverse(10.0, -6.0);
  EXPECT_DOUBLE_EQ(cmd.throttle, 0.0);
  EXPECT_DOUBLE_EQ(cmd.brake, 0.75);
  cmd = map.Inverse(0.0, 99.0);
  EXPECT_DOUBLE_EQ(cmd.throttle, 1.0);
  cmd = map.Inverse(0.0, -99.0);
  EXPECT_DOUBLE_EQ(cmd.brake, 1.0);
}

TEST(LongitudinalMap, RejectsBadTables) {
  std::string error;
  EXPECT_FALSE(LongitudinalMap::FromTables(
                   {0.0, 1.0}, {0.0, 1.0}, {{0.0, 1.0}}, {0.0, 1.0},
                   {{0.0, -1.0}, {0.0, -1.0}}, {0.0, 0.0}, &error)
                   .has_value());
  EXPECT_EQ(error, "accel_table shape != (2, 2)");
  EXPECT_FALSE(LongitudinalMap::FromTables(
                   {1.0, 0.0}, {0.0, 1.0}, {{0.0, 1.0}, {0.0, 1.0}}, {0.0, 1.0},
                   {{0.0, -1.0}, {0.0, -1.0}}, {0.0, 0.0}, &error)
                   .has_value());
  EXPECT_EQ(error, "v_bins must be increasing with at least two entries");
  EXPECT_FALSE(
      LongitudinalMap::FromYamlString("name: x\n", &error).has_value());
  EXPECT_EQ(error, "missing longitudinal_map block");
  EXPECT_FALSE(LongitudinalMap::FromYamlString("[", &error).has_value());
  EXPECT_FALSE(
      LongitudinalMap::FromYamlFile("/nonexistent.yaml", &error).has_value());
}

TEST(VehicleModel, ParsesGeometryLimitsAndFit) {
  std::string error;
  const std::optional<VehicleModel> maybe = ParseVehicleModel(kYaml, &error);
  ASSERT_TRUE(maybe.has_value()) << error;
  const VehicleModel model = maybe.value_or(VehicleModel{});
  EXPECT_EQ(model.name, "toy");
  EXPECT_DOUBLE_EQ(model.wheelbase_m, 2.5);
  EXPECT_DOUBLE_EQ(model.wheelbase_fitted_m, 2.6);
  EXPECT_DOUBLE_EQ(model.EffectiveWheelbaseM(10.0), 2.7);
  EXPECT_DOUBLE_EQ(model.limits.steer_rate_max_radps, 0.5);
  EXPECT_DOUBLE_EQ(model.limits.a_min_mps2, -5.0);
  ASSERT_TRUE(model.longitudinal_map.has_value());
  EXPECT_DOUBLE_EQ(
      model.longitudinal_map.value_or(LongitudinalMap{}).Coast(0.0), -0.5);
  // Absent fit: the geometric wheelbase and no understeer.
  const std::optional<VehicleModel> maybe_plain = ParseVehicleModel(
      "wheelbase: 3.0\nlongitudinal_map:\n  v_bins: [0.0, 1.0]\n"
      "  throttle_bins: [0.0, 1.0]\n  accel_table: [[0.0, 1.0], [0.0, 1.0]]\n"
      "  brake_bins: [0.0, 1.0]\n  decel_table: [[0.0, -1.0], [0.0, -1.0]]\n"
      "  coast_accel: [0.0, 0.0]\n",
      &error);
  ASSERT_TRUE(maybe_plain.has_value()) << error;
  const VehicleModel plain = maybe_plain.value_or(VehicleModel{});
  EXPECT_DOUBLE_EQ(plain.EffectiveWheelbaseM(30.0), 3.0);
  EXPECT_FALSE(ParseVehicleModel("- a\n", &error).has_value());
  EXPECT_FALSE(ParseVehicleModel("wheelbase: 3.0\n", &error).has_value());
}

TEST(VehicleModel, LoadsTheCommittedLincoln) {
  std::string error;
  const std::optional<VehicleModel> maybe = LoadVehicleModel(
      std::string(NUWAY_REPO_ROOT) + "/configs/vehicle/lincoln_mkz_2020.yaml",
      &error);
  ASSERT_TRUE(maybe.has_value()) << error;
  const VehicleModel model = maybe.value_or(VehicleModel{});
  EXPECT_EQ(model.name, "lincoln_mkz_2020");
  EXPECT_NEAR(model.wheelbase_m, 2.86, 1e-9);
  ASSERT_TRUE(model.longitudinal_map.has_value());
  const LongitudinalMap map =
      model.longitudinal_map.value_or(LongitudinalMap{});
  // Full throttle accelerates, full brake decelerates, coast is drag.
  for (int i = 0; i <= 12; ++i) {
    const double v = 2.5 * i;
    EXPECT_GT(map.Accel(v, 1.0), 0.0) << v;
    EXPECT_LT(map.Decel(v, 1.0), map.Coast(v)) << v;
    EXPECT_LE(map.Coast(v), 0.0) << v;
    // Inverse round trip on the throttle branch.
    const double a = 0.5 * map.Accel(v, 1.0);
    const PedalCommand cmd = map.Inverse(v, a);
    EXPECT_NEAR(map.Accel(v, cmd.throttle), a, 1e-9) << v;
  }
}

}  // namespace
}  // namespace nuway_control
