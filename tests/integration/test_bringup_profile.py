"""nuway_bringup.profile: profile YAML -> node parameters, without a ROS graph (M0 §2.10)."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from nuway_bringup import profile as bp

ROOT = Path(__file__).resolve().parents[2]


def test_committed_profiles_load_and_map() -> None:
    for name in ("m0_gt_all", "sysid"):
        prof = bp.load_profile(
            bp.resolve_profile_path(str(ROOT / f"configs/profiles/{name}.yaml"))
        )
        assert prof["profile"] == name
        wm = bp.node_params(prof, "world_manager", None, ROOT)
        assert wm["use_sim_time"] is True
        assert wm["carla.port"] == 2000
        assert Path(wm["sensors"]).is_absolute()
        assert Path(wm["vehicle"]).exists()
    sysid = bp.load_profile(ROOT / "configs/profiles/sysid.yaml")
    assert bp.controller(sysid) == "none"
    ms = bp.node_params(sysid, "map_server_node", None, ROOT)
    assert ms["town"] == "sysid_straight"
    wm = bp.node_params(sysid, "world_manager", None, ROOT)
    assert wm["carla.town"].endswith("configs/maps/sysid_straight.xodr")
    assert Path(wm["carla.town"]).exists()
    m0 = bp.load_profile(ROOT / "configs/profiles/m0_gt_all.yaml")
    assert bp.controller(m0) == "pure_pursuit"
    m1 = bp.load_profile(ROOT / "configs/profiles/m1_classical.yaml")
    assert bp.controller(m1) == "mpc"
    assert bp.prediction_source(m1) == "const_vel"
    assert bp.node_params(m1, "world_manager", None, ROOT)["carla.traffic.seed"] == 0
    assert bp.node_params(m1, "mpc_node", None, ROOT)["vehicle"].endswith(
        "lincoln_mkz_2020.yaml"
    )
    assert bp.node_params(m0, "map_server_node", None, ROOT)["town"] == "Town03"
    assert bp.use_gt(m0, "localization")


def test_defaults_then_profile_then_node_block(tmp_path: Path) -> None:
    defaults = tmp_path / "defaults.yaml"
    defaults.write_text(
        yaml.safe_dump(
            {
                "pure_pursuit_pid_node": {
                    "ros__parameters": {"kp": 0.8, "vehicle": "configs/vehicle/x.yaml"}
                }
            }
        )
    )
    prof = {
        "vehicle": "configs/vehicle/lincoln_mkz_2020.yaml",
        "pure_pursuit_pid_node": {"kp": 1.5, "noise": {"sigma": 0.1}},
    }
    params = bp.node_params(prof, "pure_pursuit_pid_node", defaults, ROOT)
    assert params["kp"] == 1.5
    assert params["noise.sigma"] == 0.1
    assert params["vehicle"] == str(ROOT / "configs/vehicle/lincoln_mkz_2020.yaml")
    assert params["use_sim_time"] is True
    # A node without a mapping still gets defaults, the block and use_sim_time.
    assert bp.node_params({}, "marker_node", defaults, ROOT) == {"use_sim_time": True}
    # Missing defaults file is fine.
    assert bp.load_defaults(tmp_path / "nope.yaml", "x") == {}


def test_helpers() -> None:
    assert bp.flatten({"a": {"b": 1, "c": {"d": [1, 2]}}, "e": "x"}) == {
        "a.b": 1,
        "a.c.d": [1, 2],
        "e": "x",
    }
    assert bp.get({"a": {"b": 2}}, "a.b") == 2
    assert bp.get({"a": {"b": 2}}, "a.z", 7) == 7
    assert bp.use_gt({}, "perception") is True
    assert bp.controller({}) == "mpc"  # M1 §3.8: the M0 controller is opt-in
    assert bp.resolve_profile_path("m0_gt_all").name == "m0_gt_all.yaml"
    assert bp.resolve_profile_path("/abs/p.yaml") == Path("/abs/p.yaml")


def test_rejects_non_mapping(tmp_path: Path) -> None:
    bad = tmp_path / "bad.yaml"
    bad.write_text("- a\n- b\n")
    with pytest.raises(ValueError, match="mapping"):
        bp.load_profile(bad)
