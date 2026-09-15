"""Headless BEV rendering (docs/02 §8): the one drawing implementation.

matplotlib on the ``Agg`` backend, no display, no ROS, no ``rclpy``; the
inputs are the plain-data :mod:`nuway_ml.viz.scene` structures that
``tools/viz/render_bag.py`` builds from a decoded MCAP and that training
renders build from tensors. Never imported by a runtime node.
"""
