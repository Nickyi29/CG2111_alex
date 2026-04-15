#!/usr/bin/env python3
"""
renderer.py - Map rendering helpers for the terminal display.

Based on the original Studio 16 renderer.
Improvements kept:
  1. High-contrast colour palette (walls bright white, free space clear)
  2. Robot shown as a directional arrow (↑↗→↘↓↙←↖) in bright yellow
  3. Path breadcrumb helper (path_display_coords) unchanged
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
# Glyphs and styles — original palette restored, contrast improved
# ===========================================================================

_GLYPH_WALL      = '\u2588'   # █  confirmed wall       (was red in original)
_GLYPH_WALL_SOFT = '\u2593'   # ▓  probable wall
_GLYPH_FRONTIER  = '\u2592'   # ▒  frontier / uncertain
_GLYPH_UNKNOWN   = '\u00b7'   # ·  unknown / unexplored
_GLYPH_FREE      = '\u25e6'   # ◦  free space

_STYLE_WALL      = 'bold bright_white'
_STYLE_WALL_SOFT = 'bright_white'
_STYLE_FRONTIER  = 'yellow'
_STYLE_UNKNOWN   = 'bright_black'
_STYLE_FREE      = 'cyan'

# Robot — directional arrow, bold bright yellow
_STYLE_ROBOT     = 'bold bright_yellow'

# Path breadcrumbs
_GLYPH_PATH      = '\u00b7'   # ·
_STYLE_PATH      = 'bold cyan'

# 8 directional arrows: E NE N NW W SW S SE
_DIRECTION_GLYPHS = [
    '\u2192',   # →  East
    '\u2197',   # ↗  North-East
    '\u2191',   # ↑  North
    '\u2196',   # ↖  North-West
    '\u2190',   # ←  West
    '\u2199',   # ↙  South-West
    '\u2193',   # ↓  South
    '\u2198',   # ↘  South-East
]

# ===========================================================================
# Visibility lookup table — original thresholds
# ===========================================================================
# BreezySLAM: 0 = wall, 127 = unknown, 255 = free

_VIS_TABLE = [
    (40,  _GLYPH_WALL,      _STYLE_WALL),
    (80,  _GLYPH_WALL_SOFT, _STYLE_WALL_SOFT),
    (120, _GLYPH_FRONTIER,  _STYLE_FRONTIER),
    (145, _GLYPH_UNKNOWN,   _STYLE_UNKNOWN),
    (256, _GLYPH_FREE,      _STYLE_FREE),
]

_VIS_LUT = np.empty(256, dtype=np.uint8)
for _i in range(256):
    for _j, (_thresh, _, _) in enumerate(_VIS_TABLE):
        if _i < _thresh:
            _VIS_LUT[_i] = _j
            break

# ===========================================================================
# Coordinate helpers — original implementation
# ===========================================================================

def mm_to_map_px(x_mm: float, y_mm: float) -> tuple[float, float]:
    """Convert BreezySLAM pose (mm) to map array indices (col, row)."""
    px_per_mm = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)
    old_col = x_mm * px_per_mm
    old_row = (MAP_SIZE_PIXELS - 1) - (y_mm * px_per_mm)
    col = old_row
    row = (MAP_SIZE_PIXELS - 1) - old_col
    return col, row


def pan_step_mm(zoom_idx: int) -> float:
    half_m = ZOOM_HALF_M[zoom_idx]
    if half_m is None:
        return 0.0
    return max(100.0, half_m * 1000.0 * PAN_STEP_FRACTION)


# ===========================================================================
# Robot arrow glyph — improvement: direction arrow instead of static ◉
# ===========================================================================

def robot_glyph(theta_deg: float) -> str:
    """Return a directional arrow for the robot heading."""
    idx = (int(round(theta_deg / 45.0)) + 2) % 8
    return _DIRECTION_GLYPHS[idx]


# ===========================================================================
# Path breadcrumb helper — original implementation
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
            sc = max(0, min(disp_cols - 1,
                            int((col - col_lo) / col_span * disp_cols)))
            sr = max(0, min(disp_rows - 1,
                            int((row - row_lo) / row_span * disp_rows)))
            coords.add((sr, sc))
    return coords


# ===========================================================================
# Map downsampling — original implementation
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
        MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
    maparray = np.rot90(np.flipud(maparray), k=1)

    samples_per_cell = 6
    r_centers = np.linspace(row_lo, row_hi,
                            disp_rows * samples_per_cell, endpoint=False)
    c_centers = np.linspace(col_lo, col_hi,
                            disp_cols * samples_per_cell, endpoint=False)
    r_idx = np.clip(r_centers.astype(np.int32), 0, MAP_SIZE_PIXELS - 1)
    c_idx = np.clip(c_centers.astype(np.int32), 0, MAP_SIZE_PIXELS - 1)
    sampled = maparray[np.ix_(r_idx, c_idx)]
    sampled = sampled.reshape(
        disp_rows, samples_per_cell, disp_cols, samples_per_cell)
    cell_min = sampled.min(axis=(1, 3))
    return _VIS_LUT[cell_min]
