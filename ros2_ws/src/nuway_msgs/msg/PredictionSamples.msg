std_msgs/Header header            # frame_id: "map"
uint32[] agent_ids                # A agents (ego excluded here; ego samples in M8 go to learned_candidates)
uint8 num_samples                 # S
uint8 num_timesteps               # T
float32 dt                        # 0.5 s
float32[] xy                      # [S][A][T][2]
float32[] yaw                     # [S][A][T]
float32[] sample_weight           # [S], sums to 1
