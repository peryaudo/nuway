"""Contact sheets: 20 frames tiled into one PNG (docs/02 §8.1)."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np
from PIL import Image

from nuway_ml.viz import style

if TYPE_CHECKING:
    from collections.abc import Sequence
    from pathlib import Path

    from numpy.typing import NDArray

FRAMES_PER_SHEET = style.SHEET_COLUMNS * style.SHEET_ROWS


def tile(
    frames: Sequence[NDArray[np.uint8]], columns: int = style.SHEET_COLUMNS
) -> NDArray[np.uint8]:
    """Tile RGB frames row-major into one image, each frame scaled by ``SHEET_SCALE``.

    Missing tiles of the last row stay background; frames must share one size.
    """
    if not frames:
        msg = "no frames to tile"
        raise ValueError(msg)
    h, w = frames[0].shape[0], frames[0].shape[1]
    th, tw = round(h * style.SHEET_SCALE), round(w * style.SHEET_SCALE)
    rows = (len(frames) + columns - 1) // columns
    sheet = Image.new("RGB", (tw * columns, th * rows), style.BACKGROUND)
    for i, frame in enumerate(frames):
        if frame.shape[:2] != (h, w):
            msg = f"frame {i} is {frame.shape[:2]}, expected {(h, w)}"
            raise ValueError(msg)
        small = Image.fromarray(frame).resize((tw, th), Image.Resampling.BOX)
        sheet.paste(small, ((i % columns) * tw, (i // columns) * th))
    return np.asarray(sheet, dtype=np.uint8)


def write_png(image: NDArray[np.uint8], path: Path) -> None:
    """Write an RGB array as a PNG with no metadata (byte-stable)."""
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(image).save(path, format="PNG")
