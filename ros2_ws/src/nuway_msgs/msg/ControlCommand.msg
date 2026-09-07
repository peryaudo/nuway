# Every tick; header.stamp = the tick consumed (the lockstep gate keys on it, 02 §2).
std_msgs/Header header
float32 accel                     # desired longitudinal accel, m/s^2
float32 steering_angle            # desired front wheel angle, rad
bool emergency_stop
