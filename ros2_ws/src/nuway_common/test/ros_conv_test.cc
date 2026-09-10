#include "nuway_common/ros_conv.h"

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "nuway_common/agents.h"
#include "nuway_common/frames.h"
#include "nuway_common/geometry.h"

namespace nuway_common {
namespace {

nuway_msgs::msg::EgoState EgoAt(double x, double y, double yaw) {
  nuway_msgs::msg::EgoState ego;
  ego.header.frame_id = kFrameMap;
  ego.pose = PoseMsg(SE2{x, y, yaw});
  ego.valid = true;
  return ego;
}

TEST(RosConvTest, AgentsToMapMatchesHandTransform) {
  // Ego at (10, 5) facing +y (yaw 90 deg). An agent 2 m ahead and 1 m to
  // the left in base_link is therefore 2 m up and 1 m to the west in map:
  // (10 - 1, 5 + 2). Its body-forward velocity (1, 0) points +y in map and
  // its yaw (0 in base_link) becomes 90 deg.
  const nuway_msgs::msg::EgoState ego = EgoAt(10.0, 5.0, kPi / 2.0);
  nuway_msgs::msg::AgentArray in;
  in.header.frame_id = kFrameBaseLink;
  nuway_msgs::msg::Agent agent;
  agent.id = 7;
  agent.pose = PoseMsg(SE2{2.0, 1.0, 0.0});
  agent.pose.position.z = 0.7;
  agent.vx = 1.0F;
  agent.vy = 0.0F;
  agent.history_len = 2;
  // 0.1 s ago the agent was 0.1 m behind its current base_link position.
  agent.history[0] = 1.9F;
  agent.history[1] = 1.0F;
  agent.history[2] = 0.0F;
  agent.history[3] = 1.8F;
  agent.history[4] = 1.0F;
  agent.history[5] = 0.1F;
  in.agents.push_back(agent);

  const nuway_msgs::msg::AgentArray out = AgentsToMap(in, ego);
  ASSERT_EQ(out.agents.size(), 1U);
  EXPECT_EQ(out.header.frame_id, kFrameMap);
  const nuway_msgs::msg::Agent& a = out.agents[0];
  EXPECT_NEAR(a.pose.position.x, 9.0, 1e-9);
  EXPECT_NEAR(a.pose.position.y, 7.0, 1e-9);
  EXPECT_NEAR(a.pose.position.z, 0.7, 1e-9);  // height is kept
  EXPECT_NEAR(QuaternionToYaw(QuaternionFromMsg(a.pose.orientation)), kPi / 2.0,
              1e-9);
  EXPECT_NEAR(a.vx, 0.0, 1e-6);
  EXPECT_NEAR(a.vy, 1.0, 1e-6);
  EXPECT_NEAR(a.history[0], 9.0, 1e-5);
  EXPECT_NEAR(a.history[1], 6.9, 1e-5);
  EXPECT_NEAR(a.history[2], kPi / 2.0, 1e-5);
  EXPECT_NEAR(a.history[3], 9.0, 1e-5);
  EXPECT_NEAR(a.history[4], 6.8, 1e-5);
  EXPECT_NEAR(a.history[5], (kPi / 2.0) + 0.1, 1e-5);
  // Untouched entries beyond history_len stay zero.
  EXPECT_EQ(a.history[6], 0.0F);
}

TEST(RosConvTest, AgentsAlreadyInMapAreReturnedUnchanged) {
  const nuway_msgs::msg::EgoState ego = EgoAt(10.0, 5.0, 1.0);
  nuway_msgs::msg::AgentArray in;
  in.header.frame_id = kFrameMap;
  nuway_msgs::msg::Agent agent;
  agent.pose = PoseMsg(SE2{3.0, 4.0, 0.5});
  agent.vx = 2.0F;
  in.agents.push_back(agent);
  const nuway_msgs::msg::AgentArray out = AgentsToMap(in, ego);
  EXPECT_NEAR(out.agents[0].pose.position.x, 3.0, 1e-12);
  EXPECT_NEAR(out.agents[0].pose.position.y, 4.0, 1e-12);
  EXPECT_EQ(out.agents[0].vx, 2.0F);
}

TEST(RosConvTest, PredictionSetRoundTripsThroughTheMessage) {
  PredictionSet set;
  set.agent_ids = {3, 9};
  set.num_samples = 1;
  set.num_timesteps = 2;
  set.dt_s = 0.5;
  set.xy = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
  set.yaw = {0.1, 0.2, 0.3, 0.4};
  set.sample_weight = {1.0};
  const PredictionSet back = PredictionSetFromMsg(PredictionSetToMsg(set));
  ASSERT_EQ(back.num_agents(), 2);
  EXPECT_EQ(back.IndexOf(9), 1);
  EXPECT_EQ(back.IndexOf(4), -1);
  const SE2 pose = back.PoseAt(0, 1, 1);
  EXPECT_NEAR(pose.x, 7.0, 1e-6);
  EXPECT_NEAR(pose.y, 8.0, 1e-6);
  EXPECT_NEAR(pose.yaw, 0.4, 1e-6);
  EXPECT_NEAR(back.TimeAt(1), 1.0, 1e-12);
}

TEST(RosConvTest, MalformedPredictionMessageBecomesAnEmptySet) {
  nuway_msgs::msg::PredictionSamples msg;
  msg.agent_ids = {1};
  msg.num_samples = 1;
  msg.num_timesteps = 3;
  msg.xy = {0.0, 0.0};  // too short
  msg.yaw = {0.0, 0.0, 0.0};
  msg.sample_weight = {1.0F};
  const PredictionSet set = PredictionSetFromMsg(msg);
  EXPECT_EQ(set.num_samples, 0);
  EXPECT_EQ(set.num_agents(), 0);
}

}  // namespace
}  // namespace nuway_common
