#!/usr/bin/env python3
"""
renderer.py - Map rendering helpers for the terminal display.

Converts the raw occupancy map bytes produced by BreezySLAM into coloured
Unicode block-character glyphs that can be displayed in a Textual widget.

BreezySLAM occupancy byte convention:
0 = confirmed wall / obstacle
127 = unknown (not yet visited)
255 = confirmed free space

The rendering pipeline:
1. A region of the map (in pixel coordinates) is identified based on the
   current zoom level and pan offset.
2. render_map_numpy() downsamples that region to the terminal display size
   using a min-sampling strategy so that thin walls are never diluted by
   surrounding free space.
3. Each sampled value is looked up in _VIS_TABLE to get a (glyph, style)
   pair for the Rich Text renderer.

IMPROVEMENT 1 (colour scheme): the palette is reworked for higher contrast.
IMPROVEMENT 3 (path overlay): path_display_coords() converts the list of
(x_mm, y_mm) path waypoints into screen cell indices.
"""

from __future__ import annotations

import numpy as np

from settings import (
    MAP_SIZE_PIXELS,
    MAP_SIZE_METERS,
    ZOOM_HALF_M,
    PAN_STEP_FRACTION,
)

# ===========================================================================
# IMPROVEMENT 1: revised glyphs and styles
# ===========================================================================

_GLYPH_WALL = '\u2588'
_GLYPH_WALL_SOFT = '\u2593'
_GLYPH_FRONTIER = '\u2592'
_GLYPH_UNKNOWN = '\u00b7'
_GLYPH_FREE = '\u2591'
_GLYPH_FREE_CLEAR = '\u25e6'
_GLYPH_ROBOT = '\u25c9'

_GLYPH_PATH = '\u00b7'
_STYLE_PATH = 'bold cyan'

_STYLE_WALL = 'bold bright_white'
_STYLE_WALL_SOFT = 'bright_white'
_STYLE_FRONTIER = 'yellow'
_STYLE_UNKNOWN = 'bright_black'
_STYLE_FREE = 'cyan'
_STYLE_FREE_CLEAR = 'bright_cyan'
_STYLE_ROBOT = 'bold bright_yellow'

_DIRECTION_GLYPHS = [
    '\u2192', '\u2197', '\u2191', '\u2196',
    '\u2190', '\u2199', '\u2193', '\u2198',
]

# ===========================================================================
# Threshold / glyph / style lookup table
# ===========================================================================

_VIS_TABLE = [
    (40, _GLYPH_WALL, _STYLE_WALL),
    (100, _GLYPH_WALL_SOFT, _STYLE_WALL_SOFT),
    (120, _GLYPH_FRONTIER, _STYLE_FRONTIER),
    (145, _GLYPH_UNKNOWN, _STYLE_UNKNOWN),
    (256, _GLYPH_FREE_CLEAR, _STYLE_FREE_CLEAR),
]

_VIS_LUT = np.empty(256, dtype=np.uint8)
for _i in range(256):
    for _j, (_thresh, _, _) in enumerate(_VIS_TABLE):
        if _i < _thresh:
            _VIS_LUT[_i] = _j
            break

# ===========================================================================
# Coordinate conversion
# ===========================================================================


def mm_to_map_px(x_mm: float, y_mm: float) -> tuple[float, float]:
    """Convert a BreezySLAM pose (mm) to map array indices (col, row)."""
    px_per_mm = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)
    old_col = x_mm * px_per_mm
    old_row = (MAP_SIZE_PIXELS - 1) - (y_mm * px_per_mm)

    col = old_row
    row = (MAP_SIZE_PIXELS - 1) - old_col
    return col, row


def pan_step_mm(zoom_idx: int) -> float:
    """Return the pan distance in mm for one key-press at the given zoom."""
    half_m = ZOOM_HALF_M[zoom_idx]
    if half_m is None:
        return 0.0
    return max(100.0, half_m * 1000.0 * PAN_STEP_FRACTION)


# ===========================================================================
# Robot heading glyph
# ===========================================================================


def robot_glyph(theta_deg: float) -> str:
    """Choose a directional arrow for the robot marker."""
    idx = (int(round(theta_deg / 45.0)) + 2) % 8
    return _DIRECTION_GLYPHS[idx]


# ===========================================================================
# IMPROVEMENT 3: path overlay helper
# ===========================================================================


def path_display_coords(
    path_pts: list[tuple[float, float]],
    col_lo: float,
    col_hi: float,
    row_lo: float,
    row_hi: float,
    disp_cols: int,
    disp_rows: int,
) -> set[tuple[int, int]]:
    """Convert path waypoints to display-cell (row, col) indices."""
    col_span = max(1e-9, col_hi - col_lo)
    row_span = max(1e-9, row_hi - row_lo)

    coords: set[tuple[int, int]] = set()
    for x_mm, y_mm in path_pts:
        col, row = mm_to_map_px(x_mm, y_mm)
        if col_lo <= col < col_hi and row_lo <= row < row_hi:
            sc = max(0, min(disp_cols - 1, int((col - col_lo) / col_span * disp_cols)))
            sr = max(0, min(disp_rows - 1, int((row - row_lo) / row_span * disp_rows)))
            coords.add((sr, sc))
    return coords


# ===========================================================================
# Vectorized map downsampling
# ===========================================================================


def render_map_numpy(
    mapbytes: bytes,
    col_lo: float,
    col_hi: float,
    row_lo: float,
    row_hi: float,
    disp_cols: int,
    disp_rows: int,
) -> np.ndarray:
    """Downsample a rectangular region of the map into a display-sized array."""
    maparray = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
        MAP_SIZE_PIXELS,
        MAP_SIZE_PIXELS,
    )

    maparray = np.rot90(np.flipud(maparray), k=1)

    samples_per_cell = 6

    r_centers = np.linspace(row_lo, row_hi, disp_rows * samples_per_cell, endpoint=False)
    c_centers = np.linspace(col_lo, col_hi, disp_cols * samples_per_cell, endpoint=False)

    r_idx = np.clip(r_centers.astype(np.int32), 0, MAP_SIZE_PIXELS - 1)
    c_idx = np.clip(c_centers.astype(np.int32), 0, MAP_SIZE_PIXELS - 1)

    sampled = maparray[np.ix_(r_idx, c_idx)]
    sampled = sampled.reshape(disp_rows, samples_per_cell, disp_cols, samples_per_cell)
    cell_min = sampled.min(axis=(1, 3))

    return _VIS_LUT[cell_min]
