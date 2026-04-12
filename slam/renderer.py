#!/usr/bin/env python3
"""
renderer.py - Map rendering helpers for the terminal display.

IMPROVEMENTS:
  - Robot body outline: draws a rotated box around the LIDAR centre
    showing the actual robot footprint (2 LIDAR diameters front and back).
  - Breadcrumb trail: path_display_coords() unchanged, works correctly.
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

_GLYPH_WALL      = '\u2588'
_GLYPH_WALL_SOFT = '\u2593'
_GLYPH_FRONTIER  = '\u2592'
_GLYPH_UNKNOWN   = '\u00b7'
_GLYPH_FREE      = '\u2591'
_GLYPH_FREE_CLEAR= '\u25e6'
_GLYPH_ROBOT     = '\u25c9'   # robot centre (LIDAR position)

_GLYPH_PATH      = '\u00b7'
_STYLE_PATH      = 'bold cyan'

_STYLE_WALL      = 'bold bright_white'
_STYLE_WALL_SOFT = 'bright_white'
_STYLE_FRONTIER  = 'yellow'
_STYLE_UNKNOWN   = 'bright_black'
_STYLE_FREE      = 'cyan'
_STYLE_FREE_CLEAR= 'bright_cyan'
_STYLE_ROBOT     = 'bold bright_yellow'
_STYLE_ROBOT_BODY= 'bright_yellow'     # robot outline

# Robot body outline characters
# We use a simple ░ block for body cells and box-drawing for edges.
_GLYPH_ROBOT_BODY   = '\u2591'   # light shade — robot interior
_GLYPH_ROBOT_EDGE_H = '\u2500'   # ─  horizontal edge
_GLYPH_ROBOT_EDGE_V = '\u2502'   # │  vertical edge
_GLYPH_ROBOT_TL     = '\u250C'   # ┌  top-left corner
_GLYPH_ROBOT_TR     = '\u2510'   # ┐  top-right corner
_GLYPH_ROBOT_BL     = '\u2514'   # └  bottom-left corner
_GLYPH_ROBOT_BR     = '\u2518'   # ┘  bottom-right corner

_DIRECTION_GLYPHS = [
    '\u2192', '\u2197', '\u2191', '\u2196',
    '\u2190', '\u2199', '\u2193', '\u2198',
]

# ===========================================================================
# Visibility lookup table
# ===========================================================================

_VIS_TABLE = [
    (40,  _GLYPH_WALL,       _STYLE_WALL),
    (100, _GLYPH_WALL_SOFT,  _STYLE_WALL_SOFT),
    (120, _GLYPH_FRONTIER,   _STYLE_FRONTIER),
    (145, _GLYPH_UNKNOWN,    _STYLE_UNKNOWN),
    (256, _GLYPH_FREE_CLEAR, _STYLE_FREE_CLEAR),
]

_VIS_LUT = np.empty(256, dtype=np.uint8)
for _i in range(256):
    for _j, (_thresh, _, _) in enumerate(_VIS_TABLE):
        if _i < _thresh:
            _VIS_LUT[_i] = _j
            break

# ===========================================================================
# Robot physical dimensions
# ===========================================================================

# RPLidar A1M8 diameter ≈ 70mm.
# Robot chassis: 2 LIDAR diameters front + 2 LIDAR diameters back from LIDAR.
# Total length = 4 * 70mm = 280mm.  Half-extent from LIDAR centre = 140mm.
# Width is approximately the same.
_ROBOT_HALF_LENGTH_MM = 140.0   # front and back from LIDAR centre
_ROBOT_HALF_WIDTH_MM  = 140.0   # left and right from LIDAR centre

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
    """Choose a directional arrow for the robot LIDAR marker."""
    idx = (int(round(theta_deg / 45.0)) + 2) % 8
    return _DIRECTION_GLYPHS[idx]


# ===========================================================================
# Robot body outline in display coordinates
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
    """
    Return a dict mapping (disp_row, disp_col) -> glyph for the robot body outline.

    The robot body is a rectangle centred on the LIDAR position.
    The rectangle is rotated by theta_deg (BreezySLAM CCW from +x).
    Only cells inside the display viewport are returned.

    The returned glyphs are box-drawing characters for corners and edges,
    and light-shade for interior cells.
    """
    # Pixel scale: map pixels per mm
    px_per_mm = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)

    # Half-extents in map pixels
    hl = _ROBOT_HALF_LENGTH_MM * px_per_mm   # forward/backward extent
    hw = _ROBOT_HALF_WIDTH_MM  * px_per_mm   # left/right extent

    # BreezySLAM theta is CCW from +x in mm space.
    # In map pixel space (col=old_row, row=MAP-old_col) the axes are rotated.
    # theta in map pixel space = -(theta_deg) + 90 degrees (due to axis swap).
    # We compute corners in mm space and convert each to map pixels.
    theta_rad = math.radians(theta_deg)
    cos_t = math.cos(theta_rad)
    sin_t = math.sin(theta_rad)

    # Robot centre in mm (approximate from map pixel position)
    # Reverse of mm_to_map_px:
    #   col = (MAP-1) - y*px_per_mm   →  y = ((MAP-1) - col) / px_per_mm
    #   row = (MAP-1) - x*px_per_mm   →  x = ((MAP-1) - row) / px_per_mm
    cx_mm = (MAP_SIZE_PIXELS - 1 - robot_row) / px_per_mm
    cy_mm = (MAP_SIZE_PIXELS - 1 - robot_col) / px_per_mm

    # 4 corners in mm space (forward=+x in robot frame, left=+y in robot frame)
    # Robot frame: forward along theta, left perpendicular CCW
    fwd = ( cos_t,  sin_t)   # forward unit vector
    lft = (-sin_t,  cos_t)   # left unit vector

    corners_mm = [
        (cx_mm + hl*fwd[0] + hw*lft[0],  cy_mm + hl*fwd[1] + hw*lft[1]),  # front-left
        (cx_mm + hl*fwd[0] - hw*lft[0],  cy_mm + hl*fwd[1] - hw*lft[1]),  # front-right
        (cx_mm - hl*fwd[0] - hw*lft[0],  cy_mm - hl*fwd[1] - hw*lft[1]),  # back-right
        (cx_mm - hl*fwd[0] + hw*lft[0],  cy_mm - hl*fwd[1] + hw*lft[1]),  # back-left
    ]

    # Convert corners to map pixel space
    corners_px = [mm_to_map_px(x, y) for x, y in corners_mm]

    # Convert map pixel corners to display cell coordinates
    col_span = max(1e-9, col_hi - col_lo)
    row_span = max(1e-9, row_hi - row_lo)

    def map_to_disp(col_px: float, row_px: float) -> tuple[float, float]:
        dc = (col_px - col_lo) / col_span * disp_cols
        dr = (row_px - row_lo) / row_span * disp_rows
        return dc, dr

    corners_disp = [map_to_disp(c, r) for c, r in corners_px]

    # Find bounding box in display space
    dcs = [c for c, r in corners_disp]
    drs = [r for c, r in corners_disp]
    dc_min = int(math.floor(min(dcs)))
    dc_max = int(math.ceil(max(dcs)))
    dr_min = int(math.floor(min(drs)))
    dr_max = int(math.ceil(max(drs)))

    cells: dict[tuple[int, int], str] = {}

    for dr in range(dr_min, dr_max + 1):
        for dc in range(dc_min, dc_max + 1):
            # Skip if outside display
            if dr < 0 or dr >= disp_rows or dc < 0 or dc >= disp_cols:
                continue

            # Determine if this display cell is inside the rotated rectangle.
            # Convert display cell centre back to mm space.
            col_px = (dc + 0.5) / disp_cols * col_span + col_lo
            row_px = (dr + 0.5) / disp_rows * row_span + row_lo

            # Back to mm
            x_mm = (MAP_SIZE_PIXELS - 1 - row_px) / px_per_mm
            y_mm = (MAP_SIZE_PIXELS - 1 - col_px) / px_per_mm

            # Transform to robot frame
            dx = x_mm - cx_mm
            dy = y_mm - cy_mm
            local_fwd  =  dx * cos_t + dy * sin_t
            local_left = -dx * sin_t + dy * cos_t

            if abs(local_fwd) <= hl and abs(local_left) <= hw:
                # Inside robot body — determine glyph by position
                on_front = abs(local_fwd  - hl) < hl * 0.25
                on_back  = abs(local_fwd  + hl) < hl * 0.25
                on_left  = abs(local_left - hw) < hw * 0.25
                on_right = abs(local_left + hw) < hw * 0.25

                on_edge = on_front or on_back or on_left or on_right

                if on_edge:
                    if on_front and on_left:
                        glyph = _GLYPH_ROBOT_TL
                    elif on_front and on_right:
                        glyph = _GLYPH_ROBOT_TR
                    elif on_back and on_left:
                        glyph = _GLYPH_ROBOT_BL
                    elif on_back and on_right:
                        glyph = _GLYPH_ROBOT_BR
                    elif on_front or on_back:
                        glyph = _GLYPH_ROBOT_EDGE_H
                    else:
                        glyph = _GLYPH_ROBOT_EDGE_V
                else:
                    glyph = _GLYPH_ROBOT_BODY

                cells[(dr, dc)] = glyph

    return cells


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
    disp_rows:  int,
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
    disp_rows:  int,
) -> np.ndarray:
    """Downsample a rectangular region of the map into a display-sized array."""
    maparray = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
        MAP_SIZE_PIXELS, MAP_SIZE_PIXELS,
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
