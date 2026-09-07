# Signals a real car's CAN bus would provide (02 §3.2).
std_msgs/Header header
float64 speed
float64 steering_angle
bool valid_steering               # false under the Leaderboard (angle is model-propagated from the command)
float64 throttle
float64 brake
int8 gear
