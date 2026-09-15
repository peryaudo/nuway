"""nuway_carla_bridge.sensor_rig: the chase camera's JPEG encoding."""

from __future__ import annotations

import io

import numpy as np
from PIL import Image

from nuway_carla_bridge.sensor_rig import encode_jpeg


def test_encode_jpeg_swaps_bgra_to_rgb() -> None:
    width, height = 8, 4
    bgra = np.zeros((height, width, 4), dtype=np.uint8)
    bgra[:, :, 2] = 200  # red channel in BGRA order
    bgra[:, :, 3] = 255
    data = encode_jpeg(bgra.tobytes(), width, height)
    assert data[:2] == b"\xff\xd8"  # JPEG SOI marker
    back = np.asarray(Image.open(io.BytesIO(data)).convert("RGB"))
    assert back.shape == (height, width, 3)
    assert back[:, :, 0].mean() > 150  # red where the BGRA red byte was
    assert back[:, :, 2].mean() < 50
