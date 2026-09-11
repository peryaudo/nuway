"""Frame ids and shared topic names (M0; prediction and planning topics M1).

Mirrors ``nuway_common/frames.h``.
"""

FRAME_MAP = "map"
FRAME_ODOM = "odom"
FRAME_BASE_LINK = "base_link"
FRAME_LIDAR_TOP = "lidar_top"
FRAME_IMU = "imu"
FRAME_GNSS = "gnss"

TOPIC_CLOCK = "/clock"
TOPIC_RESET_EVENT = "/nuway/sim/reset_event"
TOPIC_TICK_TIMEOUT = "/nuway/sim/tick_timeout"
TOPIC_VEHICLE_STATE = "/nuway/sim/vehicle_state"
TOPIC_GT_EGO_ODOM = "/nuway/gt/ego_odom"
TOPIC_GT_AGENTS = "/nuway/gt/agents"
TOPIC_GT_TRAFFIC_LIGHTS = "/nuway/gt/traffic_lights"
TOPIC_LANE_GRAPH = "/nuway/map/lane_graph"
TOPIC_ROUTE_PLAN = "/nuway/route/plan"
TOPIC_REFERENCE_LINE = "/nuway/route/reference_line"
TOPIC_ROUTE_WAYPOINTS = "/nuway/route/waypoints"
TOPIC_POSE = "/nuway/loc/pose"
TOPIC_PERCEPTION_AGENTS = "/nuway/perception/agents"
TOPIC_PERCEPTION_OCCUPANCY = "/nuway/perception/occupancy"
TOPIC_PERCEPTION_TRAFFIC_LIGHTS = "/nuway/perception/traffic_lights"
TOPIC_PREDICTION_SAMPLES = "/nuway/prediction/samples"
TOPIC_PREDICTION_FALLBACK_SAMPLES = "/nuway/prediction/fallback_samples"
TOPIC_PLANNING_BEHAVIOR = "/nuway/planning/behavior"
TOPIC_PLANNING_CANDIDATES = "/nuway/planning/candidates"
TOPIC_PLANNING_TRAJECTORY = "/nuway/planning/trajectory"
TOPIC_PLANNING_SAFE_TRAJECTORY = "/nuway/planning/safe_trajectory"
TOPIC_CONTROL_COMMAND = "/nuway/control/command"
TOPIC_CONTROL_DEBUG = "/nuway/control/debug"
TOPIC_DIAG_PREFIX = "/nuway/diag/"
TOPIC_CAMERA_INFO_FMT = "/nuway/sensors/{cam}/camera_info"
TOPIC_VIZ_CHASE_CAM = "/nuway/viz/chase_cam"  # eval.chase_cam only (docs/02 §8.3)
CHASE_CAM_ID = "cam_chase"  # the viz-only rig entry behind the chase camera
