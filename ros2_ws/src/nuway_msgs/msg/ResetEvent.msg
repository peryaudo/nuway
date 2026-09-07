# Published once per /nuway/sim/reset call, before the first tick of the new episode (02 §3.2, §7).
std_msgs/Header header            # stamp: the sim time the new episode starts at
uint32 episode_id                 # increments per reset
string town
geometry_msgs/Pose start_pose     # hero spawn pose, map frame
