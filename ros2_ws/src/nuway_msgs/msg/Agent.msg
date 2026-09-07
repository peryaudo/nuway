# One perceived or ground-truth agent (02 §4).
uint8 CLASS_UNKNOWN=0
uint8 CLASS_CAR=1
uint8 CLASS_TRUCK=2
uint8 CLASS_BICYCLE=3
uint8 CLASS_MOTORCYCLE=4
uint8 CLASS_PEDESTRIAN=5
uint8 CLASS_STATIC_OBSTACLE=6
uint8 HISTORY_LEN=20
uint32 id
uint8 class_id                    # one of CLASS_*
float32 score
geometry_msgs/Pose pose           # box center; frame per AgentArray.header.frame_id
float32 length
float32 width
float32 height
float32 vx                        # frame-aligned velocity (same frame as pose)
float32 vy
float32 yaw_rate
bool visible                      # GT producers: passed the visibility filter (M2). Learned perception: always true.
uint8 history_len                 # number of valid entries in history
float32[60] history               # [20][3] (x, y, yaw) at 0.1 s spacing, history[0] = 0.1 s ago, frame = AgentArray frame
