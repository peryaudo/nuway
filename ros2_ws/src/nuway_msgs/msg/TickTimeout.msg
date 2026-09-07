# The only wall-clock-triggered event in the stack (02 §2): the lockstep gate gave up waiting.
std_msgs/Header header            # stamp: the tick whose ControlCommand did not arrive in time
uint32 episode_id
float32 waited_s                  # wall-clock seconds the gate waited before giving up
