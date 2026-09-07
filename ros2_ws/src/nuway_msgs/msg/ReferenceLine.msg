std_msgs/Header header            # map frame
float32[] s                       # arc length, 0.5 m spacing
geometry_msgs/Point[] points
float32[] heading
float32[] curvature
float32[] speed_limit
float32[] left_bound              # lateral distance to drivable edge, positive
float32[] right_bound
uint32[] lane_id                  # lane at each point
