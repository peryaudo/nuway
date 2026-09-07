# Ego pose and body-frame motion (02 §4). frame_id: "map".
std_msgs/Header header            # frame_id: "map"
geometry_msgs/Pose pose           # base_link in map
float64 vx                        # body-frame longitudinal velocity
float64 vy                        # body-frame lateral velocity
float64 yaw_rate
float64 ax                        # body-frame longitudinal accel
float64 ay
float64 steering_angle            # front wheel angle, rad
float64[36] covariance            # 6x6 pose covariance (row-major), zeros if GT
bool valid                        # false while localization is initializing (no-input convention, 02 §2)
