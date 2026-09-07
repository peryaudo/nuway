// pybind11 bindings of nuway_common (M0). Argument and return conventions
// follow the Python twins in nuway_ml/common so the parity test reads
// one-to-one: quaternions are (x, y, z, w) lists, points are (x, y) tuples.
#include "bind_common.hpp"

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pybind11/stl.h>

#include <nuway_common/carla_conv.hpp>
#include <nuway_common/frenet.hpp>
#include <nuway_common/geometry.hpp>
#include <nuway_common/occupancy.hpp>
#include <nuway_common/tick.hpp>

namespace nuway_py {
namespace {

namespace py = pybind11;
using Point = std::pair<double, double>;
using Quat = std::array<double, 4>;
using Vec3 = std::array<double, 3>;

Quat ToQuat(const Eigen::Quaterniond& quat) {
  return Quat{quat.x(), quat.y(), quat.z(), quat.w()};
}

Eigen::Quaterniond FromQuat(const Quat& quat) {
  return {quat[3], quat[0], quat[1], quat[2]};
}

Vec3 ToVec3(const Eigen::Vector3d& vec) {
  return Vec3{vec.x(), vec.y(), vec.z()};
}

nuway_common::Vector2dList ToPoints(const std::vector<Point>& points) {
  nuway_common::Vector2dList out;
  out.reserve(points.size());
  for (const Point& point : points) {
    out.emplace_back(point.first, point.second);
  }
  return out;
}

void BindGeometry(py::module_& module) {
  using nuway_common::SE2;
  module.attr("PI") = nuway_common::kPi;
  module.def("wrap_angle", &nuway_common::WrapAngle, py::arg("angle_rad"));
  py::class_<SE2>(module, "SE2")
      .def(py::init<>())
      .def(py::init(
               [](double x, double y, double yaw) { return SE2{x, y, yaw}; }),
           py::arg("x"), py::arg("y"), py::arg("yaw"))
      .def_readwrite("x", &SE2::x)
      .def_readwrite("y", &SE2::y)
      .def_readwrite("yaw", &SE2::yaw);
  module.def(
      "compose",
      [](const SE2& a, const SE2& b) { return nuway_common::Compose(a, b); },
      py::arg("a"), py::arg("b"));
  module.def(
      "inverse", [](const SE2& a) { return nuway_common::Inverse(a); },
      py::arg("a"));
  module.def("between", &nuway_common::Between, py::arg("a"), py::arg("b"));
  module.def(
      "apply",
      [](const SE2& pose, const Point& point) {
        const Eigen::Vector2d out = nuway_common::Apply(
            pose, Eigen::Vector2d(point.first, point.second));
        return Point{out.x(), out.y()};
      },
      py::arg("pose"), py::arg("point"));
  module.def(
      "rotate",
      [](const SE2& pose, const Point& vec) {
        const Eigen::Vector2d out =
            nuway_common::Rotate(pose, Eigen::Vector2d(vec.first, vec.second));
        return Point{out.x(), out.y()};
      },
      py::arg("pose"), py::arg("vec"));
  module.def(
      "rpy_to_quaternion",
      [](double roll, double pitch, double yaw) {
        return ToQuat(nuway_common::RpyToQuaternion(roll, pitch, yaw));
      },
      py::arg("roll_rad"), py::arg("pitch_rad"), py::arg("yaw_rad"));
  module.def(
      "quaternion_to_rpy",
      [](const Quat& quat) {
        return ToVec3(nuway_common::QuaternionToRpy(FromQuat(quat)));
      },
      py::arg("quat"));
  module.def(
      "yaw_to_quaternion",
      [](double yaw) { return ToQuat(nuway_common::YawToQuaternion(yaw)); },
      py::arg("yaw_rad"));
  module.def(
      "quaternion_to_yaw",
      [](const Quat& quat) {
        return nuway_common::QuaternionToYaw(FromQuat(quat));
      },
      py::arg("quat"));
  module.def(
      "compose_se3",
      [](const Vec3& ta, const Quat& qa, const Vec3& tb, const Quat& qb) {
        nuway_common::SE3 a;
        a.translation = Eigen::Vector3d(ta[0], ta[1], ta[2]);
        a.rotation = FromQuat(qa);
        nuway_common::SE3 b;
        b.translation = Eigen::Vector3d(tb[0], tb[1], tb[2]);
        b.rotation = FromQuat(qb);
        const nuway_common::SE3 out = nuway_common::Compose(a, b);
        return std::make_pair(ToVec3(out.translation), ToQuat(out.rotation));
      },
      py::arg("translation_a"), py::arg("rotation_a"), py::arg("translation_b"),
      py::arg("rotation_b"));
  module.def(
      "inverse_se3",
      [](const Vec3& ta, const Quat& qa) {
        nuway_common::SE3 a;
        a.translation = Eigen::Vector3d(ta[0], ta[1], ta[2]);
        a.rotation = FromQuat(qa);
        const nuway_common::SE3 out = nuway_common::Inverse(a);
        return std::make_pair(ToVec3(out.translation), ToQuat(out.rotation));
      },
      py::arg("translation"), py::arg("rotation"));
}

void BindCarlaConv(py::module_& module) {
  using nuway_common::CarlaLocation;
  using nuway_common::CarlaRotation;
  py::class_<CarlaLocation>(module, "CarlaLocation")
      .def(py::init([](double x, double y, double z) {
             return CarlaLocation{x, y, z};
           }),
           py::arg("x") = 0.0, py::arg("y") = 0.0, py::arg("z") = 0.0)
      .def_readwrite("x", &CarlaLocation::x)
      .def_readwrite("y", &CarlaLocation::y)
      .def_readwrite("z", &CarlaLocation::z);
  py::class_<CarlaRotation>(module, "CarlaRotation")
      .def(py::init([](double pitch, double yaw, double roll) {
             return CarlaRotation{pitch, yaw, roll};
           }),
           py::arg("pitch") = 0.0, py::arg("yaw") = 0.0, py::arg("roll") = 0.0)
      .def_readwrite("pitch", &CarlaRotation::pitch_deg)
      .def_readwrite("yaw", &CarlaRotation::yaw_deg)
      .def_readwrite("roll", &CarlaRotation::roll_deg);
  module.def(
      "location_to_ros",
      [](const CarlaLocation& loc) {
        return ToVec3(nuway_common::LocationToRos(loc));
      },
      py::arg("loc"));
  module.def(
      "location_from_ros",
      [](const Vec3& ros) {
        return nuway_common::LocationFromRos(
            Eigen::Vector3d(ros[0], ros[1], ros[2]));
      },
      py::arg("ros"));
  module.def(
      "rotation_to_ros",
      [](const CarlaRotation& rot) {
        return ToVec3(nuway_common::RotationToRos(rot));
      },
      py::arg("rot"));
  module.def(
      "rotation_from_ros",
      [](const Vec3& rpy) {
        return nuway_common::RotationFromRos(
            Eigen::Vector3d(rpy[0], rpy[1], rpy[2]));
      },
      py::arg("rpy_rad"));
  module.def("yaw_to_ros", &nuway_common::YawToRos, py::arg("yaw_deg"));
  module.def("steer_from_ros", &nuway_common::SteerFromRos,
             py::arg("steering_angle_rad"), py::arg("max_steer_rad"));
  module.def("steer_to_ros", &nuway_common::SteerToRos, py::arg("steer"),
             py::arg("max_steer_rad"));
  module.def("yaw_from_ros", &nuway_common::YawFromRos, py::arg("yaw_rad"));
  module.def(
      "transform_to_ros",
      [](const CarlaLocation& loc, const CarlaRotation& rot) {
        const nuway_common::SE3 out = nuway_common::TransformToRos(loc, rot);
        return std::make_pair(ToVec3(out.translation), ToQuat(out.rotation));
      },
      py::arg("loc"), py::arg("rot"));
  module.def(
      "angular_velocity_to_ros",
      [](const Vec3& omega) {
        return ToVec3(nuway_common::AngularVelocityToRos(
            Eigen::Vector3d(omega[0], omega[1], omega[2])));
      },
      py::arg("omega_deg"));
}

void BindFrenet(py::module_& module) {
  using nuway_common::CartesianPoint;
  using nuway_common::FrenetPoint;
  using nuway_common::ReferenceLine;
  py::class_<FrenetPoint>(module, "FrenetPoint")
      .def(py::init([](double s, double d) { return FrenetPoint{s, d}; }),
           py::arg("s") = 0.0, py::arg("d") = 0.0)
      .def_readwrite("s", &FrenetPoint::s)
      .def_readwrite("d", &FrenetPoint::d);
  py::class_<CartesianPoint>(module, "CartesianPoint")
      .def(py::init<>())
      .def_readwrite("x", &CartesianPoint::x)
      .def_readwrite("y", &CartesianPoint::y)
      .def_readwrite("heading", &CartesianPoint::heading);
  py::class_<ReferenceLine>(module, "ReferenceLine")
      .def_static(
          "from_points",
          [](const std::vector<Point>& points) {
            return ReferenceLine::FromPoints(ToPoints(points));
          },
          py::arg("points"))
      .def_static(
          "from_samples",
          [](const std::vector<Point>& points, const std::vector<double>& s,
             const std::vector<double>& heading,
             const std::vector<double>& curvature) {
            return ReferenceLine::FromSamples(ToPoints(points), s, heading,
                                              curvature);
          },
          py::arg("points"), py::arg("s"), py::arg("heading"),
          py::arg("curvature"))
      .def_static(
          "menger_curvature",
          [](const Point& first, const Point& mid, const Point& last) {
            return ReferenceLine::MengerCurvature(
                Eigen::Vector2d(first.first, first.second),
                Eigen::Vector2d(mid.first, mid.second),
                Eigen::Vector2d(last.first, last.second));
          },
          py::arg("first"), py::arg("mid"), py::arg("last"))
      .def_property_readonly("size", &ReferenceLine::size)
      .def_property_readonly("length", &ReferenceLine::length)
      .def_property_readonly("s", &ReferenceLine::s)
      .def_property_readonly("heading", &ReferenceLine::heading)
      .def_property_readonly("curvature", &ReferenceLine::curvature)
      .def("segment_index", &ReferenceLine::SegmentIndex, py::arg("s"))
      .def("point_at", &ReferenceLine::PointAt, py::arg("s"))
      .def("heading_at", &ReferenceLine::HeadingAt, py::arg("s"))
      .def("curvature_at", &ReferenceLine::CurvatureAt, py::arg("s"))
      .def("to_cartesian", &ReferenceLine::ToCartesian, py::arg("frenet"))
      .def("to_frenet", &ReferenceLine::ToFrenet, py::arg("x"), py::arg("y"),
           py::arg("max_dist") = 1e9);
}

void BindOccupancy(py::module_& module) {
  using nuway_common::GridIndex;
  using nuway_common::GridSpec;
  py::class_<GridSpec>(module, "GridSpec")
      .def(py::init([](double resolution, double x_min, double y_min,
                       int height, int width) {
             return GridSpec{resolution, x_min, y_min, height, width};
           }),
           py::arg("resolution") = 0.5, py::arg("x_min") = -50.0,
           py::arg("y_min") = -50.0, py::arg("height") = 200,
           py::arg("width") = 200)
      .def_readwrite("resolution", &GridSpec::resolution)
      .def_readwrite("x_min", &GridSpec::x_min)
      .def_readwrite("y_min", &GridSpec::y_min)
      .def_readwrite("height", &GridSpec::height)
      .def_readwrite("width", &GridSpec::width);
  module.attr("OCCUPANCY_CHANNEL_NAMES") = [] {
    std::vector<std::string> names;
    names.reserve(nuway_common::kOccupancyChannelNames.size());
    for (const std::string_view name : nuway_common::kOccupancyChannelNames) {
      names.emplace_back(name);
    }
    return names;
  }();
  module.def(
      "world_to_grid_coord",
      [](const GridSpec& spec, double x, double y) {
        const nuway_common::GridCoord coord =
            nuway_common::WorldToGridCoord(spec, x, y);
        return Point{coord.row, coord.col};
      },
      py::arg("spec"), py::arg("x"), py::arg("y"));
  module.def(
      "world_to_grid",
      [](const GridSpec& spec, double x, double y) {
        const std::optional<GridIndex> idx =
            nuway_common::WorldToGrid(spec, x, y);
        std::optional<std::pair<int, int>> out;
        if (idx.has_value()) {
          out = std::make_pair(idx->row, idx->col);
        }
        return out;
      },
      py::arg("spec"), py::arg("x"), py::arg("y"));
  module.def(
      "grid_to_world",
      [](const GridSpec& spec, int row, int col) {
        const std::array<double, 2> out =
            nuway_common::GridToWorld(spec, row, col);
        return Point{out[0], out[1]};
      },
      py::arg("spec"), py::arg("row"), py::arg("col"));
  module.def(
      "bilinear_sample",
      [](const GridSpec& spec, const std::vector<double>& channel, double x,
         double y, double outside) {
        return nuway_common::BilinearSample(spec, channel, x, y, outside);
      },
      py::arg("spec"), py::arg("channel"), py::arg("x"), py::arg("y"),
      py::arg("outside") = 0.0);
}

void BindTick(py::module_& module) {
  module.attr("TICK_DT_S") = nuway_common::kTickDtS;
  module.def(
      "tick_index",
      [](double stamp_s) { return nuway_common::TickIndex(stamp_s); },
      py::arg("stamp_s"));
  module.def("tick_time_s", &nuway_common::TickTimeS, py::arg("k"));
  module.def(
      "tick_stamp",
      [](std::int64_t k) {
        const builtin_interfaces::msg::Time stamp = nuway_common::TickStamp(k);
        return std::make_pair(static_cast<std::int64_t>(stamp.sec),
                              static_cast<std::int64_t>(stamp.nanosec));
      },
      py::arg("k"));
  module.def("is_planning_tick", &nuway_common::IsPlanningTick, py::arg("k"));
}

}  // namespace

void BindCommon(py::module_& module) {
  BindGeometry(module);
  BindCarlaConv(module);
  BindFrenet(module);
  BindOccupancy(module);
  BindTick(module);
}

}  // namespace nuway_py
