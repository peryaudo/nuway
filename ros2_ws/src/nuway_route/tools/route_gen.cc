// route_gen (M0 task 13): writes a Leaderboard-format route XML by walking
// random successor chains through a town's lane graph. Every route is checked
// with the same RoutePlanner the stack runs, so the set is drivable by
// construction and deterministic for a seed. Positions in the file are CARLA
// convention, as the Leaderboard writes them (nuway_ml/common/routes.py
// converts back through carla_conv).
//
//   route_gen --xodr data/maps/Town03/map.xodr --town Town03 --routes 4
//             --min-length-m 1500 --spacing-m 50 --seed 3 --prefix dev03
//             --out tools/eval/routes/dev_town03.xml
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

#include <nuway_common/carla_conv.h>
#include <nuway_common/geometry.h>
#include <nuway_map/lane_graph.h>
#include <nuway_map/opendrive_parser.h>

#include "nuway_route/route_planner.h"

namespace {

struct Args {
  std::string xodr;
  std::string town = "Town03";
  std::string prefix = "route";
  std::string out;
  int routes = 4;
  double min_length_m = 1500.0;
  double spacing_m = 50.0;
  unsigned seed = 0;
};

std::optional<Args> ParseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string value = argv[i + 1];
    if (key == "--xodr") {
      args.xodr = value;
    } else if (key == "--town") {
      args.town = value;
    } else if (key == "--prefix") {
      args.prefix = value;
    } else if (key == "--out") {
      args.out = value;
    } else if (key == "--routes") {
      args.routes = static_cast<int>(std::strtol(value.c_str(), nullptr, 10));
    } else if (key == "--min-length-m") {
      args.min_length_m = std::strtod(value.c_str(), nullptr);
    } else if (key == "--spacing-m") {
      args.spacing_m = std::strtod(value.c_str(), nullptr);
    } else if (key == "--seed") {
      args.seed =
          static_cast<unsigned>(std::strtol(value.c_str(), nullptr, 10));
    } else {
      std::fprintf(stderr, "unknown argument %s\n", key.c_str());
      return std::nullopt;
    }
  }
  if (args.xodr.empty() || args.out.empty()) {
    std::fprintf(stderr,
                 "usage: route_gen --xodr <map.xodr> --out <file.xml> "
                 "[--town T] [--routes N] [--min-length-m L] "
                 "[--spacing-m S] [--seed K] [--prefix P]\n");
    return std::nullopt;
  }
  return args;
}

struct Route {
  std::vector<std::uint32_t> lane_ids;
  nuway_common::Vector3dList waypoints;  // ROS convention
  double length_m = 0.0;
};

// One random successor walk from `start`; nullopt when the walk dead-ends
// before min_length_m or the planner rejects it.
std::optional<Route> Walk(const nuway_map::LaneGraph& graph,
                          const nuway_map::Lane& start, const Args& args,
                          std::mt19937& rng) {
  Route route;
  nuway_common::Vector3dList points;
  std::set<std::uint32_t> visited;
  const nuway_map::Lane* lane = &start;
  while (route.length_m < args.min_length_m) {
    route.lane_ids.push_back(lane->id);
    visited.insert(lane->id);
    points.insert(points.end(), lane->centerline.begin(),
                  lane->centerline.end());
    route.length_m += lane->length_m;
    std::vector<const nuway_map::Lane*> candidates;
    for (const std::uint32_t id : lane->successors) {
      const nuway_map::Lane* next = graph.lane(id);
      if (next != nullptr && visited.count(next->id) == 0 &&
          next->type == nuway_map::LaneType::kDriving) {
        candidates.push_back(next);
      }
    }
    if (candidates.empty()) {
      break;
    }
    std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);
    lane = candidates[pick(rng)];
  }
  if (route.length_m < args.min_length_m || points.size() < 2) {
    return std::nullopt;
  }
  // Waypoints: 5 m into the first lane, then every spacing_m, then the end.
  double along = 0.0;
  double since_last = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double step = (points[i] - points[i - 1]).norm();
    along += step;
    since_last += step;
    if (route.waypoints.empty()) {
      if (along >= 5.0) {
        route.waypoints.push_back(points[i]);
        since_last = 0.0;
      }
    } else if (since_last >= args.spacing_m) {
      route.waypoints.push_back(points[i]);
      since_last = 0.0;
    }
  }
  if (route.waypoints.size() < 2) {
    return std::nullopt;
  }
  if ((route.waypoints.back() - points.back()).norm() > 1.0) {
    route.waypoints.push_back(points.back());
  }
  // Validate with the stack's planner from the first waypoint, heading along
  // the lane, the way run_routes.py resets the hero.
  const nuway_common::ReferenceLine* line =
      graph.reference_line(route.lane_ids.front());
  if (line == nullptr) {
    return std::nullopt;
  }
  const std::optional<nuway_common::FrenetPoint> f =
      line->ToFrenet(route.waypoints[0].x(), route.waypoints[0].y(), 5.0);
  const nuway_common::SE2 ego{route.waypoints[0].x(), route.waypoints[0].y(),
                              line->HeadingAt(f.has_value() ? f->s : 0.0)};
  nuway_common::Vector2dList goals;
  for (std::size_t i = 1; i < route.waypoints.size(); ++i) {
    goals.emplace_back(route.waypoints[i].x(), route.waypoints[i].y());
  }
  std::string error;
  const std::optional<nuway_route::RoutePlan> plan =
      nuway_route::PlanRoute(graph, ego, goals, {}, &error);
  if (!plan.has_value()) {
    std::fprintf(stderr, "  rejected: %s\n", error.c_str());
    return std::nullopt;
  }
  // The plan must be the walk itself: a route whose waypoints admit a
  // cheaper lane sequence would be driven differently from how it was made.
  if (plan->lane_ids != route.lane_ids) {
    std::fprintf(stderr, "  rejected: planned %zu lanes, walked %zu\n",
                 plan->lane_ids.size(), route.lane_ids.size());
    return std::nullopt;
  }
  return route;
}

}  // namespace

int main(int argc, char** argv) {
  const std::optional<Args> args = ParseArgs(argc, argv);
  if (!args.has_value()) {
    return 2;
  }
  std::string error;
  const std::optional<nuway_map::OpenDriveMap> map =
      nuway_map::LoadOpenDrive(args->xodr, &error);
  if (!map.has_value()) {
    std::fprintf(stderr, "%s\n", error.c_str());
    return 1;
  }
  const nuway_map::LaneGraph graph = nuway_map::LaneGraph::Build(*map);
  // Start candidates: long driving lanes (junction lanes are short).
  std::vector<const nuway_map::Lane*> starts;
  for (const nuway_map::Lane& lane : graph.lanes()) {
    if (lane.type == nuway_map::LaneType::kDriving && lane.length_m >= 30.0 &&
        !lane.successors.empty()) {
      starts.push_back(&lane);
    }
  }
  if (starts.empty()) {
    std::fprintf(stderr, "no start lanes\n");
    return 1;
  }
  std::mt19937 rng(args->seed);
  std::uniform_int_distribution<std::size_t> pick_start(0, starts.size() - 1);
  std::vector<Route> routes;
  std::set<std::uint32_t> used_starts;
  for (int attempt = 0;
       attempt < 2000 && static_cast<int>(routes.size()) < args->routes;
       ++attempt) {
    const nuway_map::Lane* start = starts[pick_start(rng)];
    if (used_starts.count(start->id) != 0) {
      continue;
    }
    std::optional<Route> route = Walk(graph, *start, *args, rng);
    if (route.has_value()) {
      used_starts.insert(start->id);
      std::fprintf(stderr, "route %zu: %zu lanes, %.0f m, %zu waypoints\n",
                   routes.size(), route->lane_ids.size(), route->length_m,
                   route->waypoints.size());
      routes.push_back(std::move(*route));
    }
  }
  if (static_cast<int>(routes.size()) < args->routes) {
    std::fprintf(stderr, "only %zu of %d routes found\n", routes.size(),
                 args->routes);
    return 1;
  }
  std::ofstream out(args->out);
  if (!out) {
    std::fprintf(stderr, "cannot write %s\n", args->out.c_str());
    return 1;
  }
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
      << "<!-- generated by route_gen (nuway_route) from " << args->xodr
      << ", seed " << args->seed << "; positions are CARLA convention -->\n"
      << "<routes>\n";
  for (std::size_t i = 0; i < routes.size(); ++i) {
    char id[64];
    std::snprintf(id, sizeof(id), "%s_%02zu", args->prefix.c_str(), i);
    out << "  <route id=\"" << id << "\" town=\"" << args->town << "\">\n"
        << "    <waypoints>\n";
    for (const Eigen::Vector3d& p : routes[i].waypoints) {
      const nuway_common::CarlaLocation loc = nuway_common::LocationFromRos(p);
      char line[128];
      std::snprintf(line, sizeof(line),
                    "      <position x=\"%.2f\" y=\"%.2f\" z=\"%.2f\"/>\n",
                    loc.x, loc.y, loc.z);
      out << line;
    }
    out << "    </waypoints>\n  </route>\n";
  }
  out << "</routes>\n";
  std::fprintf(stderr, "wrote %s (%zu routes)\n", args->out.c_str(),
               routes.size());
  return 0;
}
