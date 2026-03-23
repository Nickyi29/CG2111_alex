#!/usr/bin/env python3
"""
renderer.py - Map rendering helpers for the terminal display.

Converts the raw occupancy map bytes produced by BreezySLAM into coloured
Unicode block-character glyphs that can be displayed in a Textual widget.

BreezySLAM occupancy byte convention:
  0   = confirmed wall / obstacle
  127 = unknown (not yet visited)
  255 = confirmed free space

The rendering pipeline:
  1. A region of the map (in pixel coordinates) is identified based on the
     current zoom level and pan offset.
  2. _render_map_numpy() downsamples that region to the terminal display size
     using a min-sampling strategy so that thin walls are never diluted by
     surrounding free space.
  3. Each sampled value is looked up in _VIS_TABLE to get a (glyph, style)
     pair for the Rich Text renderer.

IMPROVEMENT 1 (colour scheme): the palette is reworked for higher contrast.
  - Walls are now bright white on a dark background instead of just red, making
    them pop clearly at a glance.
  - Free space uses a soft blue-green instead of green, reducing eye strain
    over long sessions.
  - Unknown cells keep a dim dot so explored vs unexplored is immediately
    obvious.

IMPROVEMENT 3 (path overlay): path_display_coords() converts the list of
(x_mm, y_mm) path waypoints into (screen_row, screen_col) cell indices so
ui.py can overlay a breadcrumb trail on the rendered map.
"""

from __future__ import annotations

import numpy as np

from settings import (
    MAP_SIZE_PIXELS, MAP_SIZE_METERS,
    ZOOM_HALF_M, PAN_STEP_FRACTION, LIDAR_OFFSET_DEG,
)


# ===========================================================================
# IMPROVEMENT 1: revised glyphs and styles
# ===========================================================================

# Unicode block-drawing characters used to represent each map state.
_GLYPH_WALL       = '\u2588'  # full block       -- confirmed wall
_GLYPH_WALL_SOFT  = '\u2593'  # dark shade        -- probable wall
_GLYPH_FRONTIER   = '\u2592'  # medium shade      -- frontier / mixed cell
_GLYPH_UNKNOWN    = '\u00b7'  # middle dot        -- not yet visited
_GLYPH_FREE       = '\u2591'  # light shade       -- probable free space
_GLYPH_FREE_CLEAR = '\u25e6'  # white bullet      -- confirmed free space
_GLYPH_ROBOT      = '\u25c9'  # fisheye           -- robot marker

# IMPROVEMENT 3: path breadcrumb glyph and style
_GLYPH_PATH  = '\u00b7'       # middle dot        -- path waypoint
_STYLE_PATH  = 'bold cyan'    # bright cyan dot stands out against the map

# IMPROVEMENT 1: revised styles
# Walls are bold white for maximum contrast; free space is a muted cyan-green
# that is easier on the eyes over long sessions than the original bright green.
_STYLE_WALL       = 'bold bright_white'   # was 'bold bright_red'
_STYLE_WALL_SOFT  = 'bright_white'        # was 'red'
_STYLE_FRONTIER   = 'yellow'              # unchanged
_STYLE_UNKNOWN    = 'bright_black'        # unchanged (dim dot)
_STYLE_FREE       = 'cyan'               # was 'green'
_STYLE_FREE_CLEAR = 'bright_cyan'        # was 'bright_green'
_STYLE_ROBOT      = 'bold bright_yellow'  # was 'bold cyan' -- stands out on new palette

# Direction arrows for the robot marker (8 compass octants).
# Index 0 = east (positive x), then CCW: NE, N, NW, W, SW, S, SE.
_DIRECTION_GLYPHS = ['\u2192', '\u2197', '\u2191', '\u2196',
                     '\u2190', '\u2199', '\u2193', '\u2198']


# ===========================================================================
# Threshold / glyph / style lookup table
# ===========================================================================

# Each row covers occupancy bytes up to (but not including) the threshold.
# Ordered from darkest (wall) to lightest (clear free space).
_VIS_TABLE = [
    (40,  _GLYPH_WALL,       _STYLE_WALL),
    (100, _GLYPH_WALL_SOFT,  _STYLE_WALL_SOFT),
    (120, _GLYPH_FRONTIER,   _STYLE_FRONTIER),
    (145, _GLYPH_UNKNOWN,    _STYLE_UNKNOWN),
    # (220, _GLYPH_FREE,       _STYLE_FREE),
    (256, _GLYPH_FREE_CLEAR, _STYLE_FREE_CLEAR),
]

# Pre-build a 256-element lookup array: byte value -> _VIS_TABLE index.
# This avoids a Python loop per pixel during rendering.
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
    """Convert a BreezySLAM pose (mm) to map array indices (col, row).

    BreezySLAM coordinate convention:
      - x_mm increases to the right  -> col increases to the right
      - y_mm increases upward        -> row must be flipped (row 0 is the top)

    After the standard conversion we apply the same transform used in
    render_map_numpy: vertical flip then 90-degree CCW rotation.  The net
    result is:
      new_col = (MAP_SIZE_PIXELS - 1) - y_px   (north -> col 0, left)
      new_row = (MAP_SIZE_PIXELS - 1) - x_px   (east  -> row 0, top)
    """
    px_per_mm = MAP_SIZE_PIXELS / (MAP_SIZE_METERS * 1000.0)
    old_col = x_mm * px_per_mm
    old_row = (MAP_SIZE_PIXELS - 1) - (y_mm * px_per_mm)
    # 90-degree CCW rotation
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
    """Choose a directional arrow for the robot marker.

    BreezySLAM's theta is measured counter-clockwise from the positive-x axis
    (mathematical convention).  The display is rotated 90 degrees CCW from the
    default orientation, so we add 2 octants (90 degrees) to the index.

    _DIRECTION_GLYPHS is indexed as:
      0 = east (right), 1 = NE, 2 = north (up), 3 = NW,
      4 = west (left),  5 = SW, 6 = south (down), 7 = SE
    """
    idx = (int(round(theta_deg / 45.0)) + 2) % 8
    return _DIRECTION_GLYPHS[idx]


# ===========================================================================
# IMPROVEMENT 3: path overlay helper
# ===========================================================================

def path_display_coords(
    path_pts: list[tuple[float, float]],
    col_lo: float, col_hi: float,
    row_lo: float, row_hi: float,
    disp_cols: int, disp_rows: int,
) -> set[tuple[int, int]]:
    """Convert path waypoints to display-cell (row, col) indices.

    For each (x_mm, y_mm) waypoint in path_pts, this function:
      1. Converts to map pixel coordinates using mm_to_map_px().
      2. Checks whether the point falls within the current view window.
      3. If so, maps it to a (screen_row, screen_col) terminal cell index.

    Returns a set of (sr, sc) tuples.  ui.py overlays these cells with
    _GLYPH_PATH / _STYLE_PATH after the main map is rendered.

    Parameters
    ----------
    path_pts          : list of (x_mm, y_mm) from ProcessSharedState.get_path_snapshot()
    col_lo / col_hi   : horizontal pixel bounds of the current view window
    row_lo / row_hi   : vertical pixel bounds of the current view window
    disp_cols/rows    : terminal display dimensions in characters
    """
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
    col_lo: float, col_hi: float,
    row_lo: float, row_hi: float,
    disp_cols: int, disp_rows: int,
) -> np.ndarray:
    """Downsample a rectangular region of the map into a display-sized array.

    Uses min-sampling: within each display cell the minimum occupancy value
    (i.e. the most wall-like pixel) is taken.  This ensures that thin walls
    are never diluted by surrounding free-space pixels.

    The map is rotated 90 degrees CCW before sampling, matching the coordinate
    rotation applied in mm_to_map_px().

    Returns a (disp_rows, disp_cols) uint8 array of indices into _VIS_TABLE.

    Parameters
    ----------
    mapbytes  : raw occupancy map bytes (MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)
    col_lo/hi : column pixel range of the map region to display
    row_lo/hi : row pixel range of the map region to display
    disp_cols : number of terminal columns available
    disp_rows : number of terminal rows available
    """
    maparray = np.frombuffer(mapbytes, dtype=np.uint8).reshape(
        MAP_SIZE_PIXELS, MAP_SIZE_PIXELS
    )

    # BreezySLAM stores the map with row 0 = y=0 (south).  Flip vertically
    # first so that north (y=max) moves to row 0, then rotate 90 degrees CCW
    # so that east (+x, the default robot-forward direction) ends up at the
    # top of the display.  The net transform is (y_px, x_px) ->
    # (N-1-x_px, N-1-y_px), which matches mm_to_map_px exactly.
    maparray = np.rot90(np.flipud(maparray), k=1)

    # Sample up to 6 evenly-spaced points per display cell in each dimension.
    samples_per_cell = 6

    r_centers = np.linspace(row_lo, row_hi, disp_rows * samples_per_cell,
                            endpoint=False)
    c_centers = np.linspace(col_lo, col_hi, disp_cols * samples_per_cell,
                            endpoint=False)

    # Clip to valid pixel indices.
    r_idx = np.clip(r_centers.astype(np.int32), 0, MAP_SIZE_PIXELS - 1)
    c_idx = np.clip(c_centers.astype(np.int32), 0, MAP_SIZE_PIXELS - 1)

    # Extract the full sample grid: shape (disp_rows*S, disp_cols*S).
    sampled = maparray[np.ix_(r_idx, c_idx)]

    # Reshape to (disp_rows, S, disp_cols, S) then take the minimum over
    # the two sample dimensions to preserve walls.
    sampled = sampled.reshape(disp_rows, samples_per_cell,
                              disp_cols, samples_per_cell)
    cell_min = sampled.min(axis=(1, 3))  # shape: (disp_rows, disp_cols)

    # Map each byte value to a _VIS_TABLE index via the pre-built LUT.
    return _VIS_LUT[cell_min]
