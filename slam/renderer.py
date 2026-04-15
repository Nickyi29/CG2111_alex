#!/usr/bin/env python3
"""
renderer.py - Map rendering helpers for the terminal display.

NAVIGATION-OPTIMISED RENDERING:
  Maximum contrast between walls, unknown space, and free space.
  Walls = solid block █ (bright white on dark bg = clearly visible)
  Free  = empty space   (dark background = obviously safe)
  Unknown = dim dot ·   (clearly "not yet explored")
  Robot = filled box ░ outline + arrow centre
  Path  = cyan dots ·

This makes the map instantly readable for navigation:
  "If it's blank, I can drive there."
  "If it's █, it's a wall."
  "If it's ·, I haven't been there yet."
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
# HIGH CONTRAST navigation palette
# ===========================================================================

# Walls — solid bright block, maximum visibility
_GLYPH_WALL      = '\u2588'   # █ full block
_STYLE_WALL      = 'bold white'

# Probable wall / frontier — dark block
_GLYPH_FRONTIER  = '\u2593'   # ▓ dark shade
_STYLE_FRONTIER  = 'white'

# Unknown — dim dot, clearly "unexplored"
_GLYPH_UNKNOWN   = '\u00b7'   # · middle dot
_STYLE_UNKNOWN   = 'bright_black'

# Free space — EMPTY. Nothing. Drive here.
_GLYPH_FREE      = ' '
_STYLE_FREE      = 'default'

# Robot
_GLYPH_ROBOT     = '\u25c9'   # ◉ fisheye = LIDAR centre
_STYLE_ROBOT     = 'bold bright_yellow on default'

# Robot body
_GLYPH_ROBOT_BODY= '\u2591'   # ░ light shade
_STYLE_ROBOT_BODY= 'bright_yellow'

# Path breadcrumbs
_GLYPH_PATH      = '\u2022'   # • bullet — more visible than ·
_STYLE_PATH      = 'bold cyan'

# Box drawing for robot outline
_GLYPH_ROBOT_EDGE_H = '\u2500'   # ─
_GLYPH_ROBOT_EDGE_V = '\u2502'   # │
_GLYPH_ROBOT_TL     = '\u250C'   # ┌
_GLYPH_ROBOT_TR     = '\u2510'   # ┐
_GLYPH_ROBOT_BL     = '\u2514'   # └
_GLYPH_ROBOT_BR     = '\u2518'   # ┘

_DIRECTION_GLYPHS = [
    '\u2192', '\u2197', '\u2191', '\u2196',
    '\u2190', '\u2199', '\u2193', '\u2198',
]

# ===========================================================================
# Visibility lookup table — 4 categories for maximum clarity
# ===========================================================================

# BreezySLAM: 0=wall, 127=unknown, 255=free
# Thresholds tuned for maximum navigation clarity:
#   0-79:   wall (solid █)       ← widened to catch probable walls
#   80-140: frontier (▓)         ← uncertain zone — treat as wall
#   141-200: unknown (·)         ← not yet explored
#   201-255: free (space)        ← safe to drive

_VIS_TABLE = [
    (80,  _GLYPH_WALL,     _STYLE_WALL),      # 0-79   confirmed + probable wall
    (141, _GLYPH_FRONTIER, _STYLE_FRONTIER),  # 80-140 uncertain / frontier
    (201, _GLYPH_UNKNOWN,  _STYLE_UNKNOWN),   # 141-200 unknown space
    (256, _GLYPH_FREE,     _STYLE_FREE),      # 201-255 free space
]

_VIS_LUT = np.empty(256, dtype=np.uint8)
for _i in range(256):
    for _j, (_thresh, _, _) in enumerate(_VIS_TABLE):
        if _i < _thresh:
            _VIS_LUT[_i] = _j
            break

# ===========================================================================
# Robot physical dimensions (RPLidar A1M8 ~70mm dia, 2 each side)
# ===========================================================================

# Kept intentionally smaller than true chassis (~100mm half-length).
# The body outline is a visual orientation aid only — the LIDAR already
# maps true wall distances. Making it smaller stops it covering walls/path.
_ROBOT_HALF_LENGTH_MM = 60.0
_ROBOT_HALF_WIDTH_MM  = 50.0

# ===========================================================================
# Coordinate conversion
# ===========================================================================

def mm_to_map_px(x_mm: float, y_mm: float) -> tuple[float, float]:
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


def robot_glyph(theta_deg: float) -> str:
    idx = (int(round(theta_deg / 45.0)) + 2) % 8
    return _DIRECTION_GLYPHS[idx]


# ===========================================================================
# Robot body outline
# ===========================================================================

def robot_body_cells(
    robot_col: float,
    robot_row: float,
    theta_deg: float,
    col_lo: float,
    col_hi: float,
    row_lo: float,
    row_hi: float,
    disp_cols: int,
    disp_rows:  int,
) -> dict[tuple[int, int], str]:
    px_per_mm = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)
    hl = _ROBOT_HALF_LENGTH_MM * px_per_mm
    hw = _ROBOT_HALF_WIDTH_MM  * px_per_mm

    theta_rad = math.radians(theta_deg)
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)

    cx_mm = (MAP_SIZE_PIXELS - 1 - robot_row) / px_per_mm
    cy_mm = (MAP_SIZE_PIXELS - 1 - robot_col) / px_per_mm

    fwd = ( cos_t,  sin_t)
    lft = (-sin_t,  cos_t)

    col_span = max(1e-9, col_hi - col_lo)
    row_span = max(1e-9, row_hi - row_lo)

    corners_mm = [
        (cx_mm + hl*fwd[0] + hw*lft[0], cy_mm + hl*fwd[1] + hw*lft[1]),
        (cx_mm + hl*fwd[0] - hw*lft[0], cy_mm + hl*fwd[1] - hw*lft[1]),
        (cx_mm - hl*fwd[0] - hw*lft[0], cy_mm - hl*fwd[1] - hw*lft[1]),
        (cx_mm - hl*fwd[0] + hw*lft[0], cy_mm - hl*fwd[1] + hw*lft[1]),
    ]

    corners_px   = [mm_to_map_px(x, y) for x, y in corners_mm]
    corners_disp = [
        ((c - col_lo) / col_span * disp_cols,
         (r - row_lo) / row_span * disp_rows)
        for c, r in corners_px
    ]

    dcs = [c for c, r in corners_disp]
    drs = [r for c, r in corners_disp]
    dc_min = int(math.floor(min(dcs)))
    dc_max = int(math.ceil(max(dcs)))
    dr_min = int(math.floor(min(drs)))
    dr_max = int(math.ceil(max(drs)))

    cells: dict[tuple[int, int], str] = {}

    for dr in range(dr_min, dr_max + 1):
        for dc in range(dc_min, dc_max + 1):
            if dr < 0 or dr >= disp_rows or dc < 0 or dc >= disp_cols:
                continue
            col_px = (dc + 0.5) / disp_cols * col_span + col_lo
            row_px = (dr + 0.5) / disp_rows * row_span + row_lo
            x_mm = (MAP_SIZE_PIXELS - 1 - row_px) / px_per_mm
            y_mm = (MAP_SIZE_PIXELS - 1 - col_px) / px_per_mm
            dx = x_mm - cx_mm
            dy = y_mm - cy_mm
            local_fwd  =  dx * cos_t + dy * sin_t
            local_left = -dx * sin_t + dy * cos_t

            if abs(local_fwd) <= hl and abs(local_left) <= hw:
                on_front = abs(local_fwd  - hl) < hl * 0.25
                on_back  = abs(local_fwd  + hl) < hl * 0.25
                on_left  = abs(local_left - hw) < hw * 0.25
                on_right = abs(local_left + hw) < hw * 0.25
                on_edge  = on_front or on_back or on_left or on_right

                if on_edge:
                    if on_front and on_left:    glyph = _GLYPH_ROBOT_TL
                    elif on_front and on_right: glyph = _GLYPH_ROBOT_TR
                    elif on_back and on_left:   glyph = _GLYPH_ROBOT_BL
                    elif on_back and on_right:  glyph = _GLYPH_ROBOT_BR
                    elif on_front or on_back:   glyph = _GLYPH_ROBOT_EDGE_H
                    else:                       glyph = _GLYPH_ROBOT_EDGE_V
                else:
                    glyph = _GLYPH_ROBOT_BODY

                cells[(dr, dc)] = glyph

    return cells


# ===========================================================================
# Path breadcrumbs
# ===========================================================================

def path_display_coords(
    path_pts: list[tuple[float, float]],
    col_lo: float,
    col_hi: float,
    row_lo: float,
    row_hi: float,
    disp_cols: int,
    disp_rows:  int,
) -> set[tuple[int, int]]:
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
# Nearest wall distance (for status bar navigation aid)
# ===========================================================================

def nearest_wall_mm(
    robot_x_mm: float,
    robot_y_mm: float,
    mapbytes: bytes,
    search_radius_mm: float = 1000.0,
) -> float:
    """Estimate distance to nearest wall from robot position in mm."""
    px_per_mm = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)
    rob_col, rob_row = mm_to_map_px(robot_x_mm, robot_y_mm)
    rob_c = int(round(rob_col))
    rob_r = int(round(rob_row))

    radius_px = int(search_radius_mm * px_per_mm)
    maparray = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
        MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
    maparray = np.rot90(np.flipud(maparray), k=1)

    min_dist_mm = search_radius_mm
    step = max(1, radius_px // 30)  # sample sparsely for speed

    for dr in range(-radius_px, radius_px + 1, step):
        for dc in range(-radius_px, radius_px + 1, step):
            r = rob_r + dr
            c = rob_c + dc
            if 0 <= r < MAP_SIZE_PIXELS and 0 <= c < MAP_SIZE_PIXELS:
                if maparray[r, c] < 80:   # wall threshold
                    dist_px = math.sqrt(dr*dr + dc*dc)
                    dist_mm = dist_px / px_per_mm
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
    disp_rows:  int,
) -> np.ndarray:
    maparray = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
        MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
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
