cmake_minimum_required(VERSION 3.28)
project(nuway_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
    msg/Agent.msg
    msg/AgentArray.msg
    msg/BehaviorDecision.msg
    msg/ControlCommand.msg
    msg/ControlDebug.msg
    msg/Crosswalk.msg
    msg/EgoState.msg
    msg/Lane.msg
    msg/LaneGraph.msg
    msg/NodeDiag.msg
    msg/OccupancyGridMC.msg
    msg/OdometryDelta.msg
    msg/PredictionSamples.msg
    msg/ReferenceLine.msg
    msg/ResetEvent.msg
    msg/Route.msg
    msg/StopSign.msg
    msg/TickTimeout.msg
    msg/TrafficLight.msg
    msg/TrafficLightArray.msg
    msg/TrafficLightMapping.msg
    msg/Trajectory.msg
    msg/TrajectoryCandidates.msg
    msg/TrajectoryPoint.msg
    msg/VehicleState.msg
)
set(srv_files srv/NearestLane.srv srv/Reset.srv srv/SetWeather.srv)

rosidl_generate_interfaces(
    ${PROJECT_NAME}
    ${msg_files}
    ${srv_files}
    DEPENDENCIES
    builtin_interfaces
    std_msgs
    geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
