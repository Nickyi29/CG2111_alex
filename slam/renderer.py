#!/usr/bin/env python3
"""
renderer.py - Map rendering helpers for the terminal display.

RENDERING CONVENTION:
  █  = confirmed wall        (avoid)
  ▓  = uncertain / frontier  (treat as wall)
  ·  = unexplored            (go explore)
     = empty / free space   (safe to drive — shows as blank)
  ↑→↓← etc = robot arrow    (directional, bright yellow)
  •  = breadcrumb path       (cyan)
"""

from __future__ import annotations

import math
import numpy as np

from settings import (
    MAP_SIZE_PIXELS,
    MAP_SIZE_METERS,
    ZOOM_HALF_M,
    PAN_STEP_FRACTION,
)

# ===========================================================================
# Glyphs and styles
# ===========================================================================

_GLYPH_WALL     = '\u2588'   # █  confirmed wall
_STYLE_WALL     = 'bold white'

_GLYPH_FRONTIER = '\u2593'   # ▓  uncertain / frontier
_STYLE_FRONTIER = 'white'

_GLYPH_UNKNOWN  = '\u00b7'   # ·  not yet explored
_STYLE_UNKNOWN  = 'bright_black'

_GLYPH_FREE     = ' '        #    safe to drive — empty cell
_STYLE_FREE     = 'default'

# Robot — directional arrow, bold bright yellow so it pops on any background
_STYLE_ROBOT    = 'bold bright_yellow'

# Breadcrumb path dots
_GLYPH_PATH     = '\u2022'   # •  bullet dot
_STYLE_PATH     = 'bold cyan'

# 8 directional arrows indexed 0-7 (E NE N NW W SW S SE)
_DIRECTION_GLYPHS = [
    '\u2192',   # →  East       0
    '\u2197',   # ↗  North-East 1
    '\u2191',   # ↑  North      2
    '\u2196',   # ↖  North-West 3
    '\u2190',   # ←  West       4
    '\u2199',   # ↙  South-West 5
    '\u2193',   # ↓  South      6
    '\u2198',   # ↘  South-East 7
]

# ===========================================================================
# Visibility lookup table
# ===========================================================================
# BreezySLAM values: 0 = confirmed wall, 127 = unknown, 255 = free space
#
#   0-79   → wall     █
#   80-140 → frontier ▓
#   141-200→ unknown  ·
#   201-255→ free      (blank)

_VIS_TABLE = [
    (80,  _GLYPH_WALL,     _STYLE_WALL),
    (141, _GLYPH_FRONTIER, _STYLE_FRONTIER),
    (201, _GLYPH_UNKNOWN,  _STYLE_UNKNOWN),
    (256, _GLYPH_FREE,     _STYLE_FREE),
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
    """Convert BreezySLAM pose (mm) to map array indices (col, row)."""
    px_per_mm = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)
    old_col = x_mm * px_per_mm
    old_row = (MAP_SIZE_PIXELS - 1) - (y_mm * px_per_mm)
    col = old_row
    row = (MAP_SIZE_PIXELS - 1) - old_col
    return col, row


def pan_step_mm(zoom_idx: int) -> float:
    """Pan distance in mm for one key-press at the given zoom level."""
    half_m = ZOOM_HALF_M[zoom_idx]
    if half_m is None:
        return 0.0
    return max(100.0, half_m * 1000.0 * PAN_STEP_FRACTION)


# ===========================================================================
# Robot arrow glyph
# ===========================================================================

def robot_glyph(theta_deg: float) -> str:
    """
    Return a directional arrow character for the robot's current heading.

    BreezySLAM theta is CCW from +x (east).
    We map this to one of 8 arrow characters:
      theta=0   (east)  → →
      theta=90  (north) → ↑
      theta=180 (west)  → ←
      theta=270 (south) → ↓
    The +2 offset aligns BreezySLAM's coordinate convention with the
    display orientation (north = up on screen).
    """
    idx = (int(round(theta_deg / 45.0)) + 2) % 8
    return _DIRECTION_GLYPHS[idx]


# ===========================================================================
# Path breadcrumb helper
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
    """Convert path waypoints (mm) to display cell (row, col) indices."""
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
# Nearest wall distance (for status bar)
# ===========================================================================

def nearest_wall_mm(
    robot_x_mm: float,
    robot_y_mm: float,
    mapbytes: bytes,
    search_radius_mm: float = 1000.0,
) -> float:
    """Return estimated distance in mm from robot to nearest wall."""
    px_per_mm = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)
    rob_col, rob_row = mm_to_map_px(robot_x_mm, robot_y_mm)
    rob_c = int(round(rob_col))
    rob_r = int(round(rob_row))

    radius_px = int(search_radius_mm * px_per_mm)
    maparray = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
        MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
    maparray = np.rot90(np.flipud(maparray), k=1)

    min_dist_mm = search_radius_mm
    step = max(1, radius_px // 30)

    for dr in range(-radius_px, radius_px + 1, step):
        for dc in range(-radius_px, radius_px + 1, step):
            r = rob_r + dr
            c = rob_c + dc
            if 0 <= r < MAP_SIZE_PIXELS and 0 <= c < MAP_SIZE_PIXELS:
                if maparray[r, c] < 80:
                    dist_mm = math.sqrt(dr * dr + dc * dc) / px_per_mm
                    if dist_mm < min_dist_mm:
                        min_dist_mm = dist_mm
    return min_dist_mm


# ===========================================================================
# Map downsampling
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
    """Downsample map region to display size. Returns VIS_LUT index array."""
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
