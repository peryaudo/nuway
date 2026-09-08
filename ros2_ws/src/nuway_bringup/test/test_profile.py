"""nuway_bringup.profile: profile dict -> node parameters, without files or a ROS graph."""

# ruff: noqa: PT009 -- colcon runs these with unittest discovery (no pytest in the system interpreter)
from __future__ import annotations

import unittest
from pathlib import Path

from nuway_bringup import profile as bp


class ProfileTest(unittest.TestCase):
    """Flattening, dotted lookup and the parameter layering (M0 §2.10)."""

    def test_flatten_and_get(self) -> None:
        prof = {
            "carla": {"port": 2000, "town": "Town03"},
            "use_gt": {"perception": True},
        }
        flat = bp.flatten(prof)
        self.assertEqual(flat["carla.port"], 2000)
        self.assertEqual(flat["use_gt.perception"], True)
        self.assertEqual(bp.get(prof, "carla.town"), "Town03")
        self.assertIsNone(bp.get(prof, "carla.missing"))
        self.assertEqual(bp.get(prof, "carla.missing", 7), 7)

    def test_node_params_layering(self) -> None:
        prof = {
            "carla": {"port": 2000, "town": "Town03"},
            "sensors": "configs/sensors/rig_dev.json",
            "vehicle": "configs/vehicle/lincoln_mkz_2020.yaml",
            "use_gt": {"perception": True},
            "control": {"controller": "pure_pursuit"},
            "pure_pursuit_pid_node": {"lookahead_min_m": 9.0},
        }
        root = Path("/repo")
        params = bp.node_params(prof, "pure_pursuit_pid_node", None, root)
        self.assertTrue(params["use_sim_time"])
        self.assertEqual(params["lookahead_min_m"], 9.0)
        self.assertEqual(
            params["vehicle"], "/repo/configs/vehicle/lincoln_mkz_2020.yaml"
        )
        ms = bp.node_params(prof, "map_server_node", None, root)
        self.assertEqual(ms["town"], "Town03")
        prof["carla"]["town"] = "configs/maps/sysid_straight.xodr"
        ms = bp.node_params(prof, "map_server_node", None, root)
        self.assertEqual(ms["town"], "sysid_straight")
        self.assertTrue(bp.use_gt(prof, "perception"))
        self.assertEqual(bp.controller(prof), "pure_pursuit")


if __name__ == "__main__":
    unittest.main()
