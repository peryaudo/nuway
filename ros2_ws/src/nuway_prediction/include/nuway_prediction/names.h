// Node name constants of nuway_prediction (M1).
#ifndef NUWAY_PREDICTION_NAMES_H_
#define NUWAY_PREDICTION_NAMES_H_

namespace nuway_prediction {

// The constant-velocity predictor node (M1 §3.1); runs in every profile as
// the fallback_samples producer.
constexpr const char* kConstVelNodeName = "const_vel_node";

}  // namespace nuway_prediction

#endif  // NUWAY_PREDICTION_NAMES_H_
