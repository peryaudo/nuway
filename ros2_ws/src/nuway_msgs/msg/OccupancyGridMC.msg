# Multi-channel BEV occupancy grid in base_link (02 §1, §4). Channel order is fixed:
# 0 occupied, 1 free, 2 unknown, 3 drivable, 4 height_max, 5 dynamic.
std_msgs/Header header            # frame_id: "base_link"
float32 resolution
float32 x_min
float32 y_min
uint16 height                     # rows (x)
uint16 width                      # cols (y)
uint8 num_channels
string[] channel_names            # see channel spec in docs/02_interfaces.md §4
float32[] data                    # [channel][row][col], row-major, values in [0,1] or meters for height
