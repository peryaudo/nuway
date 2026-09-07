# Static; one per OpenDRIVE signal type 1000001.
uint32 id                         # stable per town: hash of (road_id, signal_id), also used by TrafficLight.id
uint32[] affected_lane_ids        # lanes governed by this light, after configs/maps/<town>/tl_overrides.yaml
geometry_msgs/Point stop_line     # map frame, on the first affected lane's centerline
float32 heading                   # signal facing direction (OpenDRIVE hOffset resolved to map yaw)
geometry_msgs/Point bulb_position # map frame, from configs/maps/<town>/tl_bulbs.json; zero if unknown
bool from_override                # true if the association came from tl_overrides.yaml, not <validity>
