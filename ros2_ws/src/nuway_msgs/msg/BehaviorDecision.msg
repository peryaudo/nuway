uint8 LATERAL_KEEP=0
uint8 LATERAL_CHANGE_LEFT=1
uint8 LATERAL_CHANGE_RIGHT=2
uint8 LONGITUDINAL_FREE=0
uint8 LONGITUDINAL_FOLLOW=1
uint8 LONGITUDINAL_YIELD=2
uint8 LONGITUDINAL_STOP=3
std_msgs/Header header
uint8 lateral                     # one of LATERAL_*
uint32 target_lane_id
uint8 longitudinal                # one of LONGITUDINAL_*
uint32 lead_agent_id              # 0 none
float32 stop_s                    # arc length on reference line, -1 if none
float32 target_speed
string reason
