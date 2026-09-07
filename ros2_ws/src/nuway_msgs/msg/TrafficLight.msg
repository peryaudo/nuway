uint8 STATE_UNKNOWN=0
uint8 STATE_RED=1
uint8 STATE_YELLOW=2
uint8 STATE_GREEN=3
uint8 STATE_OFF=4
uint32 id                         # LaneGraph.traffic_lights[].id
uint8 state                       # one of STATE_*
geometry_msgs/Point stop_line     # map frame
uint32[] affected_lane_ids
float32 confidence                # 1.0 for GT
float32 time_in_state             # seconds since the light entered `state` as observed by the producer; -1 if unknown
float32 yellow_duration           # GT: CARLA get_yellow_time(). Learned: the configured conservative default (3 s)
bool latched                      # learned only: state is being held by the visibility latch, not observed now
