#!/usr/bin/env python3
"""
ui.py - Terminal map viewer using the Textual framework.

Based on the original Studio 16 ui.py.
Improvements kept:
  1. Robot shown as directional arrow (↑↗→↘↓↙←↖)
  2. Breadcrumb trail on by default, toggle with 't'
  3. Save map with 's'

Keyboard controls:
  + / -       Zoom in / out
  1-5         Jump to zoom level
  arrows/hjkl Pan
  c           Re-centre on robot
  p           Pause / resume SLAM
  t           Toggle breadcrumb trail
  s           Save map + path
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
)


class SlamApp(App[None]):
    """Full-screen Textual application that displays the SLAM map."""

    CSS = """
    Screen {
        background: #101418;
        color: white;
    }
    #root {
        height: 1fr;
        padding: 0 0;
    }
    #header {
        height: auto;
        content-align: left middle;
        color: white;
        background: #1d2630;
        padding: 0 1;
        text-style: bold;
    }
    #map {
        height: 1fr;
        padding: 0 0;
        content-align: center middle;
        background: #0b0f14;
    }
    #status {
        height: auto;
        color: white;
        background: #182028;
        padding: 0 1;
    }
    #help {
        height: auto;
        color: #b8c4cf;
        background: #141b22;
        padding: 0 1;
    }
    Footer {
        background: #1d2630;
    }
    """

    BINDINGS = [
        Binding('+', 'zoom_in',      'Zoom In'),
        Binding('=', 'zoom_in',      'Zoom In',   show=False),
        Binding('-', 'zoom_out',     'Zoom Out'),
        Binding('_', 'zoom_out',     'Zoom Out',  show=False),
        Binding('1', 'set_zoom(0)',  'Zoom 1',    show=False),
        Binding('2', 'set_zoom(1)',  'Zoom 2',    show=False),
        Binding('3', 'set_zoom(2)',  'Zoom 3',    show=False),
        Binding('4', 'set_zoom(3)',  'Zoom 4',    show=False),
        Binding('5', 'set_zoom(4)',  'Zoom 5',    show=False),
        Binding('left',  'pan_left',  'Pan Left',  show=False),
        Binding('h',     'pan_left',  'Pan Left',  show=False),
        Binding('right', 'pan_right', 'Pan Right', show=False),
        Binding('l',     'pan_right', 'Pan Right', show=False),
        Binding('up',    'pan_up',    'Pan Up',    show=False),
        Binding('k',     'pan_up',    'Pan Up',    show=False),
        Binding('down',  'pan_down',  'Pan Down',  show=False),
        Binding('j',     'pan_down',  'Pan Down',  show=False),
        Binding('c', 'center',       'Center'),
        Binding('p', 'pause_toggle', 'Pause'),
        Binding('t', 'path_toggle',  'Path'),
        Binding('s', 'save_map',     'Save'),
        Binding('q', 'quit',         'Quit'),
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
        self.zoom_idx  = DEFAULT_ZOOM
        self.pan_x_mm  = 0.0
        self.pan_y_mm  = 0.0
        self._last_render_key: tuple = ()
        self._cached_robot_visible   = False
        self._show_path: bool        = True   # breadcrumbs on by default
        self._save_msg: str          = ''

    def compose(self) -> ComposeResult:
        with Vertical(id='root'):
            yield Static(id='header')
            yield Static(id='map')
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
    # Keyboard actions — original implementation
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

    def action_pan_left(self)  -> None: self.pan_y_mm += pan_step_mm(self.zoom_idx)
    def action_pan_right(self) -> None: self.pan_y_mm -= pan_step_mm(self.zoom_idx)
    def action_pan_up(self)    -> None: self.pan_x_mm += pan_step_mm(self.zoom_idx)
    def action_pan_down(self)  -> None: self.pan_x_mm -= pan_step_mm(self.zoom_idx)
    def action_center(self)    -> None: self.pan_x_mm = self.pan_y_mm = 0.0

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
            parts = [f'map->{map_path}']
            path_pts = snap.get('path', [])
            if path_pts:
                path_arr  = np.array(path_pts, dtype=np.float64)
                path_path = f'slam_path_{ts}.npy'
                np.save(path_path, path_arr)
                parts.append(f'path->{path_path} ({len(path_pts)} pts)')
            self._save_msg = ' | saved: ' + ', '.join(parts)
        except Exception as exc:
            self._save_msg = f' | save FAILED: {exc}'

    def action_quit(self) -> None:
        self.pss.stop_event.set()
        self.exit()

    # -----------------------------------------------------------------------
    # Snapshot — original implementation
    # -----------------------------------------------------------------------

    def _snapshot(self) -> dict:
        error = self.pss.get_error()
        return {
            'mapbytes':     bytes(self.pss.shm.buf),
            'x_mm':         self.pss.x_mm.value,
            'y_mm':         self.pss.y_mm.value,
            'theta_deg':    self.pss.theta_deg.value,
            'valid_points': self.pss.valid_points.value,
            'status_note':  self.pss.get_status(),
            'rounds_seen':  self.pss.rounds_seen.value,
            'map_version':  self.pss.map_version.value,
            'pose_version': self.pss.pose_version.value,
            'connected':    self.pss.connected.value,
            'paused':       self.pss.paused.value,
            'stopped':      self.pss.stopped.value,
            'error_message': error if error else None,
            'path':         self.pss.get_path_snapshot(),
        }

    # -----------------------------------------------------------------------
    # Map rendering — original implementation, robot arrow added
    # -----------------------------------------------------------------------

    def _render_map_text(self, snapshot: dict) -> tuple[Text, bool]:
        try:
            map_widget = self.query_one('#map', Static)
            region     = map_widget.content_region
        except Exception:
            return Text(), False

        disp_cols = max(20, min(region.width,  MAX_RENDER_COLS))
        disp_rows = max(8,  min(region.height, MAX_RENDER_ROWS))

        mapbytes   = snapshot['mapbytes']
        robot_x_mm = snapshot['x_mm']
        robot_y_mm = snapshot['y_mm']
        theta_deg  = snapshot['theta_deg']
        px_per_m   = MAP_SIZE_PIXELS / MAP_SIZE_METERS

        rob_col, rob_row = mm_to_map_px(robot_x_mm, robot_y_mm)

        # Viewport — original logic
        zoom_half_m = ZOOM_HALF_M[self.zoom_idx]
        if zoom_half_m is None:
            col_lo, col_hi = 0.0, float(MAP_SIZE_PIXELS)
            row_lo, row_hi = 0.0, float(MAP_SIZE_PIXELS)
        else:
            cx, cy = mm_to_map_px(
                robot_x_mm + self.pan_x_mm,
                robot_y_mm + self.pan_y_mm,
            )
            half   = zoom_half_m * px_per_m
            col_lo = cx - half;  col_hi = cx + half
            row_lo = cy - half;  row_hi = cy + half

        col_span = max(1e-9, col_hi - col_lo)
        row_span = max(1e-9, row_hi - row_lo)

        # Robot position in display coords
        robot_visible = (col_lo <= rob_col < col_hi and
                         row_lo <= rob_row < row_hi)
        if robot_visible:
            robot_sc = max(0, min(disp_cols - 1,
                int((rob_col - col_lo) / col_span * disp_cols)))
            robot_sr = max(0, min(disp_rows - 1,
                int((rob_row - row_lo) / row_span * disp_rows)))
        else:
            robot_sc = robot_sr = -1

        # Breadcrumb path cells
        path_cells: set[tuple[int, int]] = set()
        if self._show_path:
            path_pts = snapshot.get('path', [])
            if path_pts:
                path_cells = path_display_coords(
                    path_pts, col_lo, col_hi, row_lo, row_hi,
                    disp_cols, disp_rows,
                )

        # Downsample map
        vis_idx = render_map_numpy(
            mapbytes, col_lo, col_hi, row_lo, row_hi, disp_cols, disp_rows)

        # Render — original run-length encoder + robot arrow on top
        text = Text(no_wrap=True)
        for sr in range(disp_rows):
            row_data  = vis_idx[sr]
            run_glyph = ''
            run_style = ''

            for sc in range(disp_cols):

                # Robot arrow — highest priority
                if robot_visible and sr == robot_sr and sc == robot_sc:
                    if run_glyph:
                        text.append(run_glyph, style=run_style)
                        run_glyph = ''
                    text.append(robot_glyph(theta_deg), style=_STYLE_ROBOT)
                    continue

                # Breadcrumb path (free/unknown cells only)
                cell_vis = int(row_data[sc])
                if (sr, sc) in path_cells and cell_vis >= 3:
                    if run_glyph:
                        text.append(run_glyph, style=run_style)
                        run_glyph = ''
                    text.append(_GLYPH_PATH, style=_STYLE_PATH)
                    continue

                # Map tile
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

        return text, robot_visible

    # -----------------------------------------------------------------------
    # Periodic refresh — original implementation
    # -----------------------------------------------------------------------

    def _refresh_view(self) -> None:
        try:
            header        = self.query_one('#header', Static)
            map_widget    = self.query_one('#map',    Static)
            status_widget = self.query_one('#status', Static)
            help_widget   = self.query_one('#help',   Static)
        except Exception:
            return

        snapshot = self._snapshot()

        state = 'PAUSED' if snapshot['paused'] else 'LIVE'
        if snapshot['error_message']:
            state = 'ERROR'
        elif snapshot['stopped'] and not snapshot['connected']:
            state = 'STOPPED'

        path_flag = 'PATH:on' if self._show_path else 'PATH:off'

        half_m = ZOOM_HALF_M[self.zoom_idx]
        if half_m is None:
            view_str = f'full map {MAP_SIZE_METERS:.1f}m x {MAP_SIZE_METERS:.1f}m'
        else:
            view_str = f'close-up {half_m * 2:.1f}m x {half_m * 2:.1f}m'

        header.update(
            f'SLAM Map | {view_str} | '
            f'Zoom {self.zoom_idx + 1}/{len(ZOOM_HALF_M)} | '
            f'{state} | {path_flag}'
        )

        region     = map_widget.content_region
        path_count = len(snapshot['path']) if self._show_path else 0
        render_key = (
            snapshot['map_version'],
            snapshot['pose_version'],
            self.zoom_idx,
            round(self.pan_x_mm, 0),
            round(self.pan_y_mm, 0),
            region.width,
            region.height,
            self._show_path,
            path_count,
        )

        if render_key != self._last_render_key:
            map_text, robot_visible = self._render_map_text(snapshot)
            self._cached_robot_visible = robot_visible
            self._last_render_key      = render_key
            map_widget.update(map_text)

        robot_flag = 'visible' if self._cached_robot_visible else 'off-screen'

        status_widget.update(
            f'Pose: x={snapshot["x_mm"] / 1000:+.2f}m, '
            f'y={snapshot["y_mm"] / 1000:+.2f}m, '
            f'theta={snapshot["theta_deg"]:.1f} deg | '
            f'Robot: {robot_flag} | '
            f'Valid points: {snapshot["valid_points"]} | '
            f'Rounds: {snapshot["rounds_seen"]} | '
            f'Status: {snapshot["status_note"]}'
            f'{self._save_msg}'
        )

        help_widget.update(
            'Keys: +/- zoom | 1-5 presets | arrows/hjkl pan | '
            'c centre | p pause | t path | s save | q quit'
        )


def run() -> None:
    SlamApp().run()
