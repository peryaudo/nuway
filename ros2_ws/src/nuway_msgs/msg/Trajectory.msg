std_msgs/Header header            # frame_id: "map" (all planning in map frame)
TrajectoryPoint[] points          # 0.1 s spacing, horizon 8 s (81 points). Every producer emits 81 points.
string source                     # "lattice", "learned", "fallback", "gt", "none" (no-input stop at the current pose, 02 §2)
uint32 candidate_id
int32 sample_index                # M8: index into PredictionSamples of the joint sample this trajectory is consistent with; -1 otherwise
