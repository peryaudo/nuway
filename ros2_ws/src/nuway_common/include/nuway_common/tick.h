// Tick index and phase of docs/02_interfaces.md §2: every stamp in the stack
// is nominally k * 0.05 s, comparisons go through TickIndex(), 10 Hz nodes act
// on even ticks, and TickBarrier implements the current-tick barrier (all
// per-tick inputs stamped k present before a node runs). Mirrored by
// nuway_ml/common/tick.py (parity-tested). Introduced in M0.
#ifndef NUWAY_COMMON_TICK_H_
#define NUWAY_COMMON_TICK_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>

namespace nuway_common {

constexpr double kTickDtS = 0.05;
constexpr std::int64_t kNanosPerSecond = 1000000000;

// Tick index of a stamp in seconds: k = round(stamp / 0.05).
inline std::int64_t TickIndex(double stamp_s) {
  return static_cast<std::int64_t>(std::llround(stamp_s / kTickDtS));
}

// Tick index of a ROS stamp.
inline std::int64_t TickIndex(const builtin_interfaces::msg::Time& stamp) {
  return TickIndex(static_cast<double>(stamp.sec) +
                   (static_cast<double>(stamp.nanosec) / 1e9));
}

// Stamp in seconds of tick k.
inline double TickTimeS(std::int64_t k) {
  return static_cast<double>(k) * kTickDtS;
}

// ROS stamp of tick k (exact: 50 ms multiples are integer nanoseconds).
inline builtin_interfaces::msg::Time TickStamp(std::int64_t k) {
  builtin_interfaces::msg::Time stamp;
  const std::int64_t nanos = k * 50000000;
  // Floor division, like Python's: a negative k (before a reset) keeps
  // nanosec in [0, 1e9) instead of wrapping the unsigned field.
  std::int64_t sec = nanos / kNanosPerSecond;
  std::int64_t rem = nanos % kNanosPerSecond;
  if (rem < 0) {
    rem += kNanosPerSecond;
    --sec;
  }
  stamp.sec = static_cast<std::int32_t>(sec);
  stamp.nanosec = static_cast<std::uint32_t>(rem);
  return stamp;
}

// 10 Hz nodes act on even ticks only; odd ticks are control-only.
inline bool IsPlanningTick(std::int64_t k) { return (k % 2) == 0; }

// Current-tick barrier over a fixed set of named per-tick inputs. Arrive() is
// called with each input's tick index; IsComplete(k) is true when every
// non-degraded input has arrived for tick k. Degrade() removes an input from
// the wait set until Reset() (docs/02 §2 Degradation); Reset() also drops
// every recorded arrival, which is what a ResetEvent requires.
class TickBarrier {
 public:
  explicit TickBarrier(std::vector<std::string> inputs)
      : inputs_(std::move(inputs)) {
    Reset();
  }

  void Arrive(const std::string& input, std::int64_t k) { latest_[input] = k; }

  bool IsComplete(std::int64_t k) const {
    return std::all_of(inputs_.begin(), inputs_.end(),
                       [this, k](const std::string& name) {
                         return degraded_.at(name) || HasArrived(name, k);
                       });
  }

  // Names of the inputs that have not (yet) arrived for tick k.
  std::vector<std::string> Missing(std::int64_t k) const {
    std::vector<std::string> out;
    for (const std::string& name : inputs_) {
      if (degraded_.at(name)) {
        continue;
      }
      if (!HasArrived(name, k)) {
        out.push_back(name);
      }
    }
    return out;
  }

  void Degrade(const std::string& input) { degraded_[input] = true; }
  bool IsDegraded(const std::string& input) const {
    return degraded_.at(input);
  }

  void Reset() {
    latest_.clear();
    for (const std::string& name : inputs_) {
      degraded_[name] = false;
    }
  }

  const std::vector<std::string>& inputs() const { return inputs_; }

 private:
  // Exactly tick k: a message for k + 1 (possible only after a TickTimeout
  // let the world move on) does not stand in for the missing k.
  bool HasArrived(const std::string& name, std::int64_t k) const {
    const auto found = latest_.find(name);
    return found != latest_.end() && found->second == k;
  }

  std::vector<std::string> inputs_;
  std::map<std::string, std::int64_t> latest_;
  std::map<std::string, bool> degraded_;
};

}  // namespace nuway_common

#endif  // NUWAY_COMMON_TICK_H_
