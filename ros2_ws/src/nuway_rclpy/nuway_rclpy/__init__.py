"""rclpy-side helpers shared by every Python node package (M0).

``nuway_ml`` never imports rclpy (docs/01 Rules); the few conversions that
need ROS message types or ``QoSProfile`` live here so that the CARLA bridge and
the GT perception twins do not depend on each other.
"""
