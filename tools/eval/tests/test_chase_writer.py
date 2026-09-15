"""chase_writer: frame paths and bytes-through writing."""

from __future__ import annotations

from pathlib import Path

from nuway_eval.chase_writer import chase_frame_path, write_chase_frame


def test_frames_are_written_by_tick(tmp_path: Path) -> None:
    assert chase_frame_path(tmp_path, 1234) == tmp_path / "chase" / "001234.jpg"
    path = write_chase_frame(tmp_path, 7, b"\xff\xd8jpeg")
    assert path == tmp_path / "chase" / "000007.jpg"
    assert path.read_bytes() == b"\xff\xd8jpeg"
