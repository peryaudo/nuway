"""nuway C++ libraries bound with pybind11 (M0).

Re-exports ``nuway_py._core``. Available only after ``colcon build`` with
``ros2_ws/install/setup.bash`` sourced (``setup_env.sh`` does that). The one
sanctioned import from ``ros2_ws`` into ``tools/`` and ``tests/``
(``docs/01_directory_structure.md`` Rules).
"""

from nuway_py import _core
from nuway_py._core import *  # noqa: F403  -- the extension module is the API

__all__ = [name for name in dir(_core) if not name.startswith("_")]
