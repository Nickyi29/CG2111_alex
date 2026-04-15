#!/usr/bin/env python3
"""
ui.py - Terminal map viewer using the Textual framework.

FEATURES:
  - Robot shown as a directional arrow (↑ ↗ → ↘ ↓ ↙ ← ↖) in bright yellow.
    The arrow rotates with the robot heading so you always know which way
    the robot is facing.
  - Auto-follow: map centres on robot at all times unless manually panned.
    Press 'f' to toggle, 'c' to re-centre and re-enable.
  - Wall proximity warning bar: green/yellow/red distance to nearest wall.
  - Breadcrumb trail: cyan dots showing where the robot has been. Toggle 't'.
  - 6 zoom levels: 1=full map, 2=arena, 3=navigation(default),
                   4=close, 5=detail, 6=arm positioning.

Map legend (always shown in help bar):
  █ = wall    ▓ = uncertain    · = unexplored    (blank) = SAFE to drive
  ↑ = robot (arrow points in direction robot is facing)
  • = path breadcrumb

Keyboard controls:
  + / -       Zoom in / out
  1-6         Jump to zoom level
  arrows/hjkl Pan (disables auto-follow)
  c           Re-centre on robot + re-enable auto-follow
  f           Toggle auto-follow
  p           Pause / resume SLAM
  t           Toggle breadcrumb trail
  s           Save map + path to .npy files
  q           Quit
"""

from __future__ import annotations

import datetime
import multiprocessing

import numpy as np
from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.widgets import Footer, Static

from settings import (
    MAP_SIZE_PIXELS,
    MAP_SIZE_METERS,
    ZOOM_HALF_M,
    DEFAULT_ZOOM,
    UI_REFRESH_HZ,
    MAX_RENDER_COLS,
    MAX_RENDER_ROWS,
)
from shared_state import ProcessSharedState
from slam_process import run_slam_process
from renderer import (
    _VIS_TABLE,
    _STYLE_ROBOT,
    _STYLE_PATH,
    _GLYPH_PATH,
    mm_to_map_px,
    pan_step_mm,
    robot_glyph,
    render_map_numpy,
    path_display_coords,
    nearest_wall_mm,
)


class SlamApp(App[None]):
    """Full-screen Textual SLAM map viewer."""

    CSS = """
    Screen { background: #0a0d10; color: white; }
    #root  { height: 1fr; padding: 0; }

    #header {
        height: auto;
        color: ansi_bright_white;
        background: #1a2535;
        padding: 0 1;
        text-style: bold;
    }
    #map {
        height: 1fr;
        content-align: center middle;
        background: #0a0d10;
        padding: 0;
    }
    #wall_warning {
        height: auto;
        padding: 0 1;
        text-style: bold;
    }
    #status {
        height: auto;
        color: white;
        background: #111820;
        padding: 0 1;
    }
    #help {
        height: auto;
        color: #7a8fa0;
        background: #0c1218;
        padding: 0 1;
    }
    Footer { background: #1a2535; }
    """

    BINDINGS = [
        Binding('+',     'zoom_in',       'Zoom In'),
        Binding('=',     'zoom_in',       'Zoom In',    show=False),
        Binding('-',     'zoom_out',      'Zoom Out'),
        Binding('_',     'zoom_out',      'Zoom Out',   show=False),
        Binding('1',     'set_zoom(0)',   'Z1:full',    show=False),
        Binding('2',     'set_zoom(1)',   'Z2:arena',   show=False),
        Binding('3',     'set_zoom(2)',   'Z3:nav',     show=False),
        Binding('4',     'set_zoom(3)',   'Z4:close',   show=False),
        Binding('5',     'set_zoom(4)',   'Z5:detail',  show=False),
        Binding('6',     'set_zoom(5)',   'Z6:arm',     show=False),
        Binding('left',  'pan_left',      'Pan ←',      show=False),
        Binding('h',     'pan_left',      'Pan ←',      show=False),
        Binding('right', 'pan_right',     'Pan →',      show=False),
        Binding('l',     'pan_right',     'Pan →',      show=False),
        Binding('up',    'pan_up',        'Pan ↑',      show=False),
        Binding('k',     'pan_up',        'Pan ↑',      show=False),
        Binding('down',  'pan_down',      'Pan ↓',      show=False),
        Binding('j',     'pan_down',      'Pan ↓',      show=False),
        Binding('c',     'center',        'Centre'),
        Binding('f',     'follow_toggle', 'Follow'),
        Binding('p',     'pause_toggle',  'Pause'),
        Binding('t',     'path_toggle',   'Path'),
        Binding('s',     'save_map',      'Save'),
        Binding('q',     'quit',          'Quit'),
    ]

    def __init__(self) -> None:
        super().__init__()
        self.pss = ProcessSharedState()
        self.slam_proc = multiprocessing.Process(
            target=run_slam_process,
            args=(self.pss,),
            name='slam-process',
            daemon=True,
        )
        self.zoom_idx   = DEFAULT_ZOOM
        self.pan_x_mm   = 0.0
        self.pan_y_mm   = 0.0
        self._tracking  = True   # auto-follow robot position
        self._show_path = True   # breadcrumb trail on by default
        self._last_render_key: tuple = ()
        self._cached_robot_visible   = False
        self._cached_wall_mm: float  = 9999.0
        self._save_msg  = ''

    # -----------------------------------------------------------------------
    # Layout
    # -----------------------------------------------------------------------

    def compose(self) -> ComposeResult:
        with Vertical(id='root'):
            yield Static(id='header')
            yield Static(id='map')
            yield Static(id='wall_warning')
            yield Static(id='status')
            yield Static(id='help')
            yield Footer()

    def on_mount(self) -> None:
        self.slam_proc.start()
        self.set_interval(1.0 / UI_REFRESH_HZ, self._refresh_view)

    def on_unmount(self) -> None:
        self.pss.stop_event.set()
        if self.slam_proc.is_alive():
            self.slam_proc.join(timeout=3.0)
        if self.slam_proc.is_alive():
            self.slam_proc.terminate()
        self.pss.cleanup()

    # -----------------------------------------------------------------------
    # Keyboard actions
    # -----------------------------------------------------------------------

    def action_zoom_in(self) -> None:
        self.zoom_idx = min(self.zoom_idx + 1, len(ZOOM_HALF_M) - 1)
        if ZOOM_HALF_M[self.zoom_idx] is None:
            self.pan_x_mm = self.pan_y_mm = 0.0

    def action_zoom_out(self) -> None:
        self.zoom_idx = max(self.zoom_idx - 1, 0)
        if ZOOM_HALF_M[self.zoom_idx] is None:
            self.pan_x_mm = self.pan_y_mm = 0.0

    def action_set_zoom(self, idx: str) -> None:
        idx_int = int(idx)
        if 0 <= idx_int < len(ZOOM_HALF_M):
            self.zoom_idx = idx_int
        if ZOOM_HALF_M[self.zoom_idx] is None:
            self.pan_x_mm = self.pan_y_mm = 0.0

    def _disable_tracking(self) -> None:
        self._tracking = False

    def action_pan_left(self)  -> None:
        self._disable_tracking()
        self.pan_y_mm += pan_step_mm(self.zoom_idx)

    def action_pan_right(self) -> None:
        self._disable_tracking()
        self.pan_y_mm -= pan_step_mm(self.zoom_idx)

    def action_pan_up(self) -> None:
        self._disable_tracking()
        self.pan_x_mm += pan_step_mm(self.zoom_idx)

    def action_pan_down(self) -> None:
        self._disable_tracking()
        self.pan_x_mm -= pan_step_mm(self.zoom_idx)

    def action_center(self) -> None:
        """Re-centre on robot and re-enable auto-follow."""
        self.pan_x_mm  = 0.0
        self.pan_y_mm  = 0.0
        self._tracking = True

    def action_follow_toggle(self) -> None:
        self._tracking = not self._tracking
        if self._tracking:
            self.pan_x_mm = self.pan_y_mm = 0.0

    def action_pause_toggle(self) -> None:
        self.pss.paused.value = not self.pss.paused.value

    def action_path_toggle(self) -> None:
        self._show_path       = not self._show_path
        self._last_render_key = ()

    def action_save_map(self) -> None:
        try:
            snap    = self._snapshot()
            ts      = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            map_arr = np.frombuffer(
                snap['mapbytes'], dtype=np.uint8,
            ).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS).copy()
            map_path = f'slam_map_{ts}.npy'
            np.save(map_path, map_arr)
            parts = [f'map→{map_path}']
            path_pts = snap.get('path', [])
            if path_pts:
                path_arr  = np.array(path_pts, dtype=np.float64)
                path_path = f'slam_path_{ts}.npy'
                np.save(path_path, path_arr)
                parts.append(f'path→{path_path} ({len(path_pts)} pts)')
            self._save_msg = '  [saved: ' + ', '.join(parts) + ']'
        except Exception as exc:
            self._save_msg = f'  [save FAILED: {exc}]'

    def action_quit(self) -> None:
        self.pss.stop_event.set()
        self.exit()

    # -----------------------------------------------------------------------
    # Shared state snapshot
    # -----------------------------------------------------------------------

    def _snapshot(self) -> dict:
        error = self.pss.get_error()
        return {
            'mapbytes':      bytes(self.pss.shm.buf),
            'x_mm':          self.pss.x_mm.value,
            'y_mm':          self.pss.y_mm.value,
            'theta_deg':     self.pss.theta_deg.value,
            'valid_points':  self.pss.valid_points.value,
            'status_note':   self.pss.get_status(),
            'rounds_seen':   self.pss.rounds_seen.value,
            'map_version':   self.pss.map_version.value,
            'pose_version':  self.pss.pose_version.value,
            'connected':     self.pss.connected.value,
            'paused':        self.pss.paused.value,
            'stopped':       self.pss.stopped.value,
            'error_message': error if error else None,
            'path':          self.pss.get_path_snapshot(),
        }

    # -----------------------------------------------------------------------
    # Viewport — auto-follow lives here
    # -----------------------------------------------------------------------

    def _get_viewport(
        self, robot_x_mm: float, robot_y_mm: float
    ) -> tuple[float, float, float, float]:
        """Return (col_lo, col_hi, row_lo, row_hi) in map pixel space."""
        zoom_half_m = ZOOM_HALF_M[self.zoom_idx]

        if zoom_half_m is None:
            # Full map view — no panning needed
            return 0.0, float(MAP_SIZE_PIXELS), 0.0, float(MAP_SIZE_PIXELS)

        half     = zoom_half_m * (MAP_SIZE_PIXELS / MAP_SIZE_METERS)
        margin   = half  # keep robot from going off the clamped edge

        if self._tracking:
            # Always centre on robot
            cx, cy = mm_to_map_px(robot_x_mm, robot_y_mm)
        else:
            cx, cy = mm_to_map_px(
                robot_x_mm + self.pan_x_mm,
                robot_y_mm + self.pan_y_mm,
            )

        # Clamp so viewport never goes outside the map array
        cx = max(margin, min(MAP_SIZE_PIXELS - margin, cx))
        cy = max(margin, min(MAP_SIZE_PIXELS - margin, cy))

        return cx - half, cx + half, cy - half, cy + half

    # -----------------------------------------------------------------------
    # Map rendering
    # -----------------------------------------------------------------------

    def _render_map_text(
        self, snapshot: dict
    ) -> tuple[Text, bool, float]:
        """
        Render the occupancy map to a Rich Text object.

        Returns (text, robot_visible, nearest_wall_mm).

        Rendering priority per cell:
          1. Robot arrow  — always on top
          2. Breadcrumb   — only in free/unknown cells (not over walls)
          3. Map tile     — wall / frontier / unknown / free
        """
        try:
            map_widget = self.query_one('#map', Static)
            region     = map_widget.content_region
        except Exception:
            return Text(), False, 9999.0

        disp_cols = max(20, min(region.width,  MAX_RENDER_COLS))
        disp_rows = max(8,  min(region.height, MAX_RENDER_ROWS))

        mapbytes   = snapshot['mapbytes']
        robot_x_mm = snapshot['x_mm']
        robot_y_mm = snapshot['y_mm']
        theta_deg  = snapshot['theta_deg']

        rob_col, rob_row = mm_to_map_px(robot_x_mm, robot_y_mm)
        col_lo, col_hi, row_lo, row_hi = self._get_viewport(
            robot_x_mm, robot_y_mm)

        col_span = max(1e-9, col_hi - col_lo)
        row_span = max(1e-9, row_hi - row_lo)

        # --- Robot position in display coordinates ---
        robot_visible = (
            col_lo <= rob_col < col_hi and row_lo <= rob_row < row_hi)
        if robot_visible:
            robot_sc = max(0, min(disp_cols - 1,
                int((rob_col - col_lo) / col_span * disp_cols)))
            robot_sr = max(0, min(disp_rows - 1,
                int((rob_row - row_lo) / row_span * disp_rows)))
        else:
            robot_sc = robot_sr = -1

        # --- Breadcrumb cells ---
        path_cells: set[tuple[int, int]] = set()
        if self._show_path:
            path_pts = snapshot.get('path', [])
            if path_pts:
                path_cells = path_display_coords(
                    path_pts, col_lo, col_hi, row_lo, row_hi,
                    disp_cols, disp_rows,
                )

        # --- Nearest wall distance ---
        wall_dist = nearest_wall_mm(robot_x_mm, robot_y_mm, mapbytes)

        # --- Downsample occupancy map ---
        vis_idx = render_map_numpy(
            mapbytes, col_lo, col_hi, row_lo, row_hi, disp_cols, disp_rows)

        # --- Build Rich Text row by row ---
        text = Text(no_wrap=True)

        for sr in range(disp_rows):
            row_data  = vis_idx[sr]
            run_glyph = ''
            run_style = ''

            for sc in range(disp_cols):

                # Priority 1 — Robot directional arrow
                if robot_visible and sr == robot_sr and sc == robot_sc:
                    if run_glyph:
                        text.append(run_glyph, style=run_style)
                        run_glyph = ''
                    text.append(robot_glyph(theta_deg), style=_STYLE_ROBOT)
                    continue

                # Priority 2 — Breadcrumb dot (free/unknown space only)
                cell_vis = int(row_data[sc])
                if (sr, sc) in path_cells and cell_vis >= 2:
                    if run_glyph:
                        text.append(run_glyph, style=run_style)
                        run_glyph = ''
                    text.append(_GLYPH_PATH, style=_STYLE_PATH)
                    continue

                # Priority 3 — Map tile (run-length encoded for speed)
                _, glyph, style = _VIS_TABLE[cell_vis]
                if style == run_style:
                    run_glyph += glyph
                else:
                    if run_glyph:
                        text.append(run_glyph, style=run_style)
                    run_glyph = glyph
                    run_style = style

            if run_glyph:
                text.append(run_glyph, style=run_style)
            if sr != disp_rows - 1:
                text.append('\n')

        return text, robot_visible, wall_dist

    # -----------------------------------------------------------------------
    # Periodic UI refresh
    # -----------------------------------------------------------------------

    def _refresh_view(self) -> None:
        try:
            header        = self.query_one('#header',       Static)
            map_widget    = self.query_one('#map',          Static)
            wall_widget   = self.query_one('#wall_warning', Static)
            status_widget = self.query_one('#status',       Static)
            help_widget   = self.query_one('#help',         Static)
        except Exception:
            return

        snapshot = self._snapshot()

        # --- State label ---
        if snapshot['error_message']:
            state = 'ERROR'
        elif snapshot['stopped'] and not snapshot['connected']:
            state = 'STOPPED'
        elif snapshot['paused']:
            state = 'PAUSED'
        else:
            state = 'LIVE'

        # --- Zoom label ---
        half_m = ZOOM_HALF_M[self.zoom_idx]
        if half_m is None:
            view_str = f'full {MAP_SIZE_METERS}m map'
        else:
            zoom_labels = [
                'full map', 'arena overview', 'navigation',
                'close up', 'detail', 'arm positioning',
            ]
            label = (zoom_labels[self.zoom_idx]
                     if self.zoom_idx < len(zoom_labels) else '')
            view_str = f'{half_m * 2:.1f}m × {half_m * 2:.1f}m  {label}'

        track_flag = 'TRACK:on' if self._tracking else 'TRACK:off (c=re-enable)'
        path_flag  = 'PATH:on'  if self._show_path else 'PATH:off'

        header.update(
            f' SLAM  │  {view_str}  │  zoom {self.zoom_idx + 1}/{len(ZOOM_HALF_M)}'
            f'  │  {state}  │  {track_flag}  │  {path_flag}'
        )

        # --- Render map (only when something has changed) ---
        region     = map_widget.content_region
        path_count = len(snapshot['path']) if self._show_path else 0
        render_key = (
            snapshot['map_version'],
            snapshot['pose_version'],
            self.zoom_idx,
            self._tracking,
            round(self.pan_x_mm, 0) if not self._tracking else 0,
            round(self.pan_y_mm, 0) if not self._tracking else 0,
            region.width,
            region.height,
            self._show_path,
            path_count,
        )

        if render_key != self._last_render_key:
            map_text, robot_visible, wall_dist = self._render_map_text(snapshot)
            self._cached_robot_visible = robot_visible
            self._cached_wall_mm       = wall_dist
            self._last_render_key      = render_key
            map_widget.update(map_text)

        # --- Wall proximity warning ---
        w = self._cached_wall_mm
        if w < 200:
            wall_style = 'bold red'
            wall_label = '⚠  WALL'
        elif w < 500:
            wall_style = 'bold yellow'
            wall_label = '!  WALL'
        else:
            wall_style = 'bold green'
            wall_label = '   WALL'

        wall_str = (
            f'{wall_label}: ---  (building map...)'
            if w >= 9000
            else f'{wall_label}: {w:4.0f} mm'
        )
        wall_widget.update(Text(wall_str, style=wall_style))

        # --- Status bar ---
        robot_str = (
            'robot: visible'
            if self._cached_robot_visible
            else 'robot: OFF-SCREEN — press c'
        )
        status_widget.update(
            f'x={snapshot["x_mm"] / 1000:+.2f}m  '
            f'y={snapshot["y_mm"] / 1000:+.2f}m  '
            f'θ={snapshot["theta_deg"]:.0f}°  │  '
            f'{robot_str}  │  '
            f'pts:{snapshot["valid_points"]}  '
            f'rounds:{snapshot["rounds_seen"]}  │  '
            f'{snapshot["status_note"]}'
            f'{self._save_msg}'
        )

        # --- Help / legend bar ---
        help_widget.update(
            '█=wall  ▓=uncertain  ·=unexplored  (blank)=SAFE  '
            '↑=robot arrow  •=path  │  '
            '+/- zoom  1-6 preset  hjkl/arrows pan  '
            'c=centre  f=follow  t=path  s=save  q=quit'
        )


def run() -> None:
    SlamApp().run()
