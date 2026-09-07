uint8 STATUS_OK=0
uint8 STATUS_WARN=1
uint8 STATUS_ERROR=2
std_msgs/Header header
string node
float32 cycle_ms
float32 input_age_ms              # now minus the stamp of the newest input
uint8 status                      # one of STATUS_*
string message
