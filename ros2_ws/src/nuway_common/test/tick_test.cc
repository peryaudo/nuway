#include "nuway_common/tick.h"

#include <gtest/gtest.h>

namespace nuway_common {
namespace {

TEST(TickTest, TickIndexRoundsAccumulatedFloatError) {
  EXPECT_EQ(TickIndex(0.0), 0);
  EXPECT_EQ(TickIndex(0.05), 1);
  EXPECT_EQ(TickIndex(1.0000000003), 20);
  EXPECT_EQ(TickIndex(0.9999999997), 20);
  EXPECT_EQ(TickIndex(12345.65), 246913);
}

TEST(TickTest, TickStampRoundTrips) {
  for (std::int64_t k = 0; k < 100000; k += 997) {
    EXPECT_EQ(TickIndex(TickStamp(k)), k);
  }
  const builtin_interfaces::msg::Time stamp = TickStamp(21);
  EXPECT_EQ(stamp.sec, 1);
  EXPECT_EQ(stamp.nanosec, 50000000U);
}

TEST(TickTest, NegativeTickStampsKeepNanosecInRange) {
  const builtin_interfaces::msg::Time stamp = TickStamp(-1);
  EXPECT_EQ(stamp.sec, -1);
  EXPECT_EQ(stamp.nanosec, 950000000U);
  EXPECT_EQ(TickIndex(stamp), -1);
}

TEST(TickTest, ArrivalIsForTheExactTick) {
  TickBarrier barrier({"pose"});
  barrier.Arrive("pose", 5);
  EXPECT_FALSE(barrier.IsComplete(4));  // k + 1 does not stand in for k
  EXPECT_TRUE(barrier.IsComplete(5));
}

TEST(TickTest, PlanningTicksAreEven) {
  EXPECT_TRUE(IsPlanningTick(0));
  EXPECT_FALSE(IsPlanningTick(1));
  EXPECT_TRUE(IsPlanningTick(42));
}

TEST(TickTest, BarrierCompletesWhenEveryInputArrivedForTheTick) {
  TickBarrier barrier({"pose", "agents"});
  EXPECT_FALSE(barrier.IsComplete(4));
  barrier.Arrive("pose", 4);
  EXPECT_FALSE(barrier.IsComplete(4));
  EXPECT_EQ(barrier.Missing(4).size(), 1U);
  barrier.Arrive("agents", 4);
  EXPECT_TRUE(barrier.IsComplete(4));
  EXPECT_FALSE(barrier.IsComplete(5));
  barrier.Degrade("agents");
  barrier.Arrive("pose", 5);
  EXPECT_TRUE(barrier.IsComplete(5));
  EXPECT_TRUE(barrier.IsDegraded("agents"));
  EXPECT_FALSE(barrier.AllDegraded());
  barrier.Degrade("pose");
  EXPECT_TRUE(barrier.AllDegraded());
  EXPECT_TRUE(barrier.IsComplete(6));  // nothing left to wait for
  barrier.Reset();
  EXPECT_FALSE(barrier.IsDegraded("agents"));
  EXPECT_FALSE(barrier.AllDegraded());
  EXPECT_FALSE(barrier.IsComplete(5));
}

}  // namespace
}  // namespace nuway_common
