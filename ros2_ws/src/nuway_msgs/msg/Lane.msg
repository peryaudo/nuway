uint8 TYPE_DRIVING=0
uint8 TYPE_SHOULDER=1
uint8 TYPE_PARKING=2
uint8 TYPE_BIDIRECTIONAL=3
uint8 TYPE_OTHER=4
uint32 id
uint32 road_id
int32 lane_id_odr
uint8 type                        # one of TYPE_*
geometry_msgs/Point[] centerline  # >= 2 points, <= 2 m spacing
float32[] width                   # per point
uint32[] successors
uint32[] predecessors
uint32 left_neighbor              # 0 = none
uint32 right_neighbor
bool left_change_allowed
bool right_change_allowed
float32 speed_limit               # m/s, 0 if unknown
