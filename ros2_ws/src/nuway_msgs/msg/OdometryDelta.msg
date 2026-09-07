# Relative motion between LiDAR keyframes (M5).
std_msgs/Header header            # stamp: LiDAR keyframe k; frame_id: "base_link"
builtin_interfaces/Time prev_stamp # keyframe k-1 the delta is measured from
geometry_msgs/Transform delta     # T_{k-1 -> k}: base_link at k-1 -> base_link at k
float64[36] covariance            # 6x6 (x, y, z, roll, pitch, yaw), from the ICP Hessian (M5 §2.1)
