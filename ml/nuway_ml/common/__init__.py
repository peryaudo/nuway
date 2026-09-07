"""Python twins of ``nuway_common`` (M0): geometry, Frenet, occupancy, tick, CARLA conversion.

Every module here mirrors its C++ header one-to-one (same names modulo case, same
argument order) and is parity-tested through ``nuway_py``
(``tests/integration/test_geometry_parity.py``). Nothing here imports torch at
module scope (``docs/03_style_and_conventions.md`` §6.1).
"""
