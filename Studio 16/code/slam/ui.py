#!/usr/bin/env python3
"""
ui.py - Terminal map viewer using the Textual framework.

Displays the BreezySLAM occupancy map as a colour Unicode block-character
grid in the terminal.  The map updates in real time as the robot moves.

Keyboard controls
-----------------
  + / =     Zoom in (show a smaller area in more detail)
  - / _     Zoom out (zoom level 1 shows the full map)
  1-5       Jump directly to zoom levels 1-5
  arrows    Pan the zoomed view
  h j k l   Pan (vi-style: left, down, up, right)
  c         Re-centre the view on the robot
  p         Pause / resume SLAM updates
  t         Toggle path breadcrumb trail on/off       (IMPROVEMENT 3)
  s         Save map + path to disk (.npy files)       (IMPROVEMENT 4)
  q         Quit (also Ctrl-C)

Map legend
----------
  bold white full block  wall / confirmed obstacle       (IMPROVEMENT 1)
  white      dark shade  near a wall                     (IMPROVEMENT 1)
  yellow     mid shade   frontier (mixed free/occupied)
  dark grey  middle dot  unknown / not yet visited
  cyan       white dot   confirmed free space             (IMPROVEMENT 1)
  yellow     fisheye     robot estimated position         (IMPROVEMENT 1)
  bold cyan  middle dot  path breadcrumb waypoint         (IMPROVEMENT 3)
"""

from __future__ import annotations

import multiprocessing
import datetime
import os

import numpy as np
from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Vertical
from textual.widgets import Footer, Static

from settings import (
    MAP_SIZE_PIXELS, MAP_SIZE_METERS,
    ZOOM_HALF_M, DEFAULT_ZOOM,
    UI_REFRESH_HZ, MAX_RENDER_COLS, MAX_RENDER_ROWS,
)
from shared_state import ProcessSharedState
from slam_process import run_slam_process
from renderer import (
    _VIS_TABLE,
    _STYLE_ROBOT, _STYLE_PATH,
    _GLYPH_WALL,       _STYLE_WALL,
    _GLYPH_WALL_SOFT,  _STYLE_WALL_SOFT,
    _GLYPH_FRONTIER,   _STYLE_FRONTIER,
    _GLYPH_UNKNOWN,    _STYLE_UNKNOWN,
    _GLYPH_FREE_CLEAR, _STYLE_FREE_CLEAR,
    _GLYPH_PATH,                                  # IMPROVEMENT 3
    mm_to_map_px, pan_step_mm, robot_glyph,
    render_map_numpy,
    path_display_coords,                           # IMPROVEMENT 3
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
        Binding('+',     'zoom_in',        'Zoom In'),
        Binding('=',     'zoom_in',        'Zoom In',    show=False),
        Binding('-',     'zoom_out',       'Zoom Out'),
        Binding('_',     'zoom_out',       'Zoom Out',   show=False),
        Binding('1',     'set_zoom(0)',    'Zoom 1',     show=False),
        Binding('2',     'set_zoom(1)',    'Zoom 2',     show=False),
        Binding('3',     'set_zoom(2)',    'Zoom 3',     show=False),
        Binding('4',     'set_zoom(3)',    'Zoom 4',     show=False),
        Binding('5',     'set_zoom(4)',    'Zoom 5',     show=False),
        Binding('left',  'pan_left',      'Pan Left',   show=False),
        Binding('h',     'pan_left',      'Pan Left',   show=False),
        Binding('right', 'pan_right',     'Pan Right',  show=False),
        Binding('l',     'pan_right',     'Pan Right',  show=False),
        Binding('up',    'pan_up',        'Pan Up',     show=False),
        Binding('k',     'pan_up',        'Pan Up',     show=False),
        Binding('down',  'pan_down',      'Pan Down',   show=False),
        Binding('j',     'pan_down',      'Pan Down',   show=False),
        Binding('c',     'center',        'Center'),
        Binding('p',     'pause_toggle',  'Pause'),
        Binding('t',     'path_toggle',   'Path'),       # IMPROVEMENT 3
        Binding('s',     'save_map',      'Save'),       # IMPROVEMENT 4
        Binding('q',     'quit',          'Quit'),
    ]

    def __init__(self) -> None:
        super().__init__()
        # Shared state object passed to the SLAM process.
        self.pss = ProcessSharedState()
        # SLAM runs in a separate child process so it has its own GIL.
        self.slam_proc = multiprocessing.Process(
            target=run_slam_process,
            args=(self.pss,),
            name='slam-process',
            daemon=True,
        )
        self.zoom_idx = DEFAULT_ZOOM
        self.pan_x_mm = 0.0
        self.pan_y_mm = 0.0
        # Cache the last render key so we skip redraws when nothing changed.
        self._last_render_key: tuple = ()
        self._cached_robot_visible = False

        # IMPROVEMENT 3: path breadcrumb trail toggle (on by default)
        self._show_path: bool = True

        # IMPROVEMENT 4: last save message shown in the status bar
        self._save_msg: str = ''

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
        # Signal the SLAM process to stop, then wait for it to exit cleanly.
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

    def action_pan_left(self) -> None:
        self.pan_y_mm += pan_step_mm(self.zoom_idx)

    def action_pan_right(self) -> None:
        self.pan_y_mm -= pan_step_mm(self.zoom_idx)

    def action_pan_up(self) -> None:
        self.pan_x_mm += pan_step_mm(self.zoom_idx)

    def action_pan_down(self) -> None:
        self.pan_x_mm -= pan_step_mm(self.zoom_idx)

    def action_center(self) -> None:
        self.pan_x_mm = self.pan_y_mm = 0.0

    def action_pause_toggle(self) -> None:
        self.pss.paused.value = not self.pss.paused.value

    # IMPROVEMENT 3: toggle path trail visibility
    def action_path_toggle(self) -> None:
        self._show_path = not self._show_path
        # Force a re-render on the next tick.
        self._last_render_key = ()

    # IMPROVEMENT 4: save map and path to .npy files on disk
    def action_save_map(self) -> None:
        """Save the current occupancy map and path to timestamped .npy files.

        Files are written to the current working directory (the slam/ folder
        if you ran slam.py from there).  Reload them later with numpy.load().

        Map format  : uint8 array of shape (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
                      Values: 0 = wall, 127 = unknown, 255 = free space.
        Path format : float64 array of shape (N, 2)
                      Columns: [x_mm, y_mm].
        """
        try:
            snap = self._snapshot()
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

            # Save the occupancy map.
            map_arr = np.frombuffer(snap['mapbytes'], dtype=np.uint8).reshape(
                MAP_SIZE_PIXELS, MAP_SIZE_PIXELS
            ).copy()  # copy() because frombuffer returns a read-only view
            map_path = f'slam_map_{ts}.npy'
            np.save(map_path, map_arr)

            # Save the path (may be empty at startup).
            path_pts = snap.get('path', [])
            msg_parts = [f'map→{map_path}']
            if path_pts:
                path_arr = np.array(path_pts, dtype=np.float64)
                path_path = f'slam_path_{ts}.npy'
                np.save(path_path, path_arr)
                msg_parts.append(f'path→{path_path} ({len(path_pts)} pts)')

            self._save_msg = '  |  saved: ' + ', '.join(msg_parts)
        except Exception as exc:
            self._save_msg = f'  |  save FAILED: {exc}'

    def action_quit(self) -> None:
        self.pss.stop_event.set()
        self.exit()

    # -----------------------------------------------------------------------
    # Read a snapshot from shared memory
    # -----------------------------------------------------------------------

    def _snapshot(self) -> dict:
        """Read all shared state into a plain dict for safe single-threaded use.

        bytes(self.pss.shm.buf) makes a one-time copy of the 1 MB map so the
        rendering code can work on a stable snapshot without locking.

        IMPROVEMENT 2 & 3: also reads the path ring buffer.
        """
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
            'path':          self.pss.get_path_snapshot(),   # IMPROVEMENT 2 & 3
        }

    # -----------------------------------------------------------------------
    # Map rendering
    # -----------------------------------------------------------------------

    def _render_map_text(self, snapshot: dict) -> tuple[Text, bool]:
        """Render the occupancy map as a Rich Text object.

        Returns (text, robot_visible) where robot_visible indicates whether
        the robot marker is within the current view window.

        IMPROVEMENT 3: path breadcrumb cells are overlaid on the rendered map
        when self._show_path is True.
        """
        try:
            map_widget = self.query_one('#map', Static)
            region = map_widget.content_region
        except Exception:
            return Text(), False

        disp_cols = max(20, min(region.width,  MAX_RENDER_COLS))
        disp_rows = max(8,  min(region.height, MAX_RENDER_ROWS))

        mapbytes    = snapshot['mapbytes']
        robot_x_mm  = snapshot['x_mm']
        robot_y_mm  = snapshot['y_mm']
        px_per_m    = MAP_SIZE_PIXELS / MAP_SIZE_METERS

        rob_col, rob_row = mm_to_map_px(robot_x_mm, robot_y_mm)

        # Compute the pixel bounds of the view window.
        zoom_half_m = ZOOM_HALF_M[self.zoom_idx]
        if zoom_half_m is None:
            col_lo, col_hi = 0.0, float(MAP_SIZE_PIXELS)
            row_lo, row_hi = 0.0, float(MAP_SIZE_PIXELS)
        else:
            cx, cy = mm_to_map_px(
                robot_x_mm + self.pan_x_mm,
                robot_y_mm + self.pan_y_mm,
            )
            half = zoom_half_m * px_per_m
            col_lo = cx - half
            col_hi = cx + half
            row_lo = cy - half
            row_hi = cy + half

        col_span = max(1e-9, col_hi - col_lo)
        row_span = max(1e-9, row_hi - row_lo)

        # Determine if the robot is within the view window and where to draw it.
        robot_visible = (col_lo <= rob_col < col_hi and
                         row_lo <= rob_row < row_hi)
        if robot_visible:
            robot_sc = max(0, min(disp_cols - 1,
                int((rob_col - col_lo) / col_span * disp_cols)))
            robot_sr = max(0, min(disp_rows - 1,
                int((rob_row - row_lo) / row_span * disp_rows)))
        else:
            robot_sc = robot_sr = -1

        # Downsample the map region to display resolution.
        vis_idx = render_map_numpy(
            mapbytes, col_lo, col_hi, row_lo, row_hi, disp_cols, disp_rows,
        )

        # IMPROVEMENT 3: compute path cell coordinates for overlay.
        path_cells: set[tuple[int, int]] = set()
        if self._show_path:
            path_pts = snapshot.get('path', [])
            if path_pts:
                path_cells = path_display_coords(
                    path_pts, col_lo, col_hi, row_lo, row_hi,
                    disp_cols, disp_rows,
                )

        # Build the Rich Text object row by row.
        # Consecutive cells with the same style are grouped into a single
        # append() call to reduce Rich overhead.
        text = Text(no_wrap=True)
        for sr in range(disp_rows):
            row_data = vis_idx[sr]
            run_glyph = ''
            run_style = ''

            for sc in range(disp_cols):
                # --- robot marker (highest priority) ---
                if robot_visible and sr == robot_sr and sc == robot_sc:
                    if run_glyph:
                        text.append(run_glyph, style=run_style)
                        run_glyph = ''
                    text.append(
                        robot_glyph(snapshot['theta_deg']),
                        style=_STYLE_ROBOT,
                    )
                    continue

                # IMPROVEMENT 3: path breadcrumb dot (second priority)
                # Only draw on free / unknown cells so walls show through.
                if (sr, sc) in path_cells:
                    cell_vis = int(row_data[sc])
                    # Indices 0-2 in _VIS_TABLE are wall/frontier -- skip those
                    # so the path dot doesn't hide wall information.
                    if cell_vis >= 3:
                        if run_glyph:
                            text.append(run_glyph, style=run_style)
                            run_glyph = ''
                        text.append(_GLYPH_PATH, style=_STYLE_PATH)
                        continue

                # --- normal map cell ---
                _, glyph, style = _VIS_TABLE[int(row_data[sc])]
                if style == run_style:
                    run_glyph += glyph
                else:
                    if run_glyph:
                        text.append(run_glyph, style=run_style)
                    run_glyph = glyph
                    run_style = style

            # Flush the last run on this row.
            if run_glyph:
                text.append(run_glyph, style=run_style)
            if sr != disp_rows - 1:
                text.append('\n')

        return text, robot_visible

    # -----------------------------------------------------------------------
    # Periodic refresh
    # -----------------------------------------------------------------------

    def _refresh_view(self) -> None:
        """Called by Textual at UI_REFRESH_HZ; reads shared state and redraws."""
        try:
            header        = self.query_one('#header',  Static)
            map_widget    = self.query_one('#map',     Static)
            status_widget = self.query_one('#status',  Static)
            help_widget   = self.query_one('#help',    Static)
        except Exception:
            return

        snapshot = self._snapshot()

        # Determine the overall state label shown in the header.
        state = 'PAUSED' if snapshot['paused'] else 'LIVE'
        if snapshot['error_message']:
            state = 'ERROR'
        elif snapshot['stopped'] and not snapshot['connected']:
            state = 'STOPPED'

        # IMPROVEMENT 3: append path toggle state to header.
        path_flag = 'PATH:on' if self._show_path else 'PATH:off'

        # Describe the current view window.
        half_m = ZOOM_HALF_M[self.zoom_idx]
        if half_m is None:
            view_str = (f'full map {MAP_SIZE_METERS:.1f}m x '
                        f'{MAP_SIZE_METERS:.1f}m')
            pan_text = 'centered'
        else:
            view_str = f'close-up {half_m * 2:.1f}m x {half_m * 2:.1f}m'
            if abs(self.pan_x_mm) < 0.5 and abs(self.pan_y_mm) < 0.5:
                pan_text = 'centered'
            else:
                pan_text = (f'{self.pan_x_mm / 1000:+.2f},'
                            f'{self.pan_y_mm / 1000:+.2f}m')

        header.update(
            f'SLAM Map | View: {view_str} | '
            f'Zoom {self.zoom_idx + 1}/{len(ZOOM_HALF_M)} | '
            f'{state} | {path_flag}'              # IMPROVEMENT 3
        )

        # Only re-render the map when something that affects the image changed.
        # IMPROVEMENT 3: include path_count and _show_path in the render key
        # so the map redraws when the path grows or visibility is toggled.
        region = map_widget.content_region
        path_count = snapshot['path'].__len__() if self._show_path else 0
        render_key = (
            snapshot['map_version'],
            snapshot['pose_version'],
            self.zoom_idx,
            round(self.pan_x_mm, 0),
            round(self.pan_y_mm, 0),
            region.width,
            region.height,
            self._show_path,                      # IMPROVEMENT 3
            path_count,                           # IMPROVEMENT 3
        )
        if render_key != self._last_render_key:
            map_text, robot_visible = self._render_map_text(snapshot)
            self._cached_robot_visible = robot_visible
            self._last_render_key = render_key
            map_widget.update(map_text)

        robot_visible = self._cached_robot_visible

        # Status bar: pose, scan quality, pan offset, save message, warnings.
        path_pts = snapshot.get('path', [])
        status_line = (
            f'Pose x={snapshot["x_mm"]:6.0f}mm  '
            f'y={snapshot["y_mm"]:6.0f}mm  '
            f'th={snapshot["theta_deg"]:+6.1f}deg | '
            f'valid={snapshot["valid_points"]:3d} | '
            f'pan={pan_text} | '
            f'path={len(path_pts)}pts | '          # IMPROVEMENT 2 & 3
            f'{snapshot["status_note"]}'
            f'{self._save_msg}'                    # IMPROVEMENT 4
        )
        if not robot_visible:
            status_line += ' | robot off-screen'
        if snapshot['error_message']:
            status_line += f' | ERROR: {snapshot["error_message"]}'
        status_widget.update(status_line)

        # Clear the save message after it has been shown for a few cycles.
        if self._save_msg:
            # Reuse the render_key tick: clear after the key changes again.
            # Simpler: just clear it next cycle via a counter.
            if not hasattr(self, '_save_msg_ticks'):
                self._save_msg_ticks = 0
            self._save_msg_ticks += 1
            if self._save_msg_ticks >= UI_REFRESH_HZ * 4:   # show for ~4 s
                self._save_msg = ''
                self._save_msg_ticks = 0
        else:
            self._save_msg_ticks = 0

        # Help bar: map legend and key hints (updated to reflect new palette).
        help_widget.update(
            'Legend: '
            f'[{_STYLE_WALL}]{_GLYPH_WALL} wall[/]  '
            f'[{_STYLE_WALL_SOFT}]{_GLYPH_WALL_SOFT} obstacle[/]  '
            f'[{_STYLE_FRONTIER}]{_GLYPH_FRONTIER} mixed[/]  '
            f'[{_STYLE_UNKNOWN}]{_GLYPH_UNKNOWN} unknown[/]  '
            f'[{_STYLE_FREE_CLEAR}]{_GLYPH_FREE_CLEAR} free[/]  '
            f'[{_STYLE_ROBOT}]\u25c9 robot[/]  '
            f'[{_STYLE_PATH}]{_GLYPH_PATH} path[/]  |  '   # IMPROVEMENT 3
            'Keys: +/- zoom  1-5 jump  arrows/hjkl pan  '
            'c center  p pause  t path  s save  q quit'     # IMPROVEMENT 3 & 4
        )


def run() -> None:
    """Launch the SLAM map viewer.  Called from slam.py."""
    try:
        import rich   # noqa: F401
        import textual  # noqa: F401
    except ImportError:
        print('[slam] ERROR: textual is not installed.')
        print('  Run:  bash ../install_slam.sh  or activate the environment.')
        raise SystemExit(1)

    SlamApp().run()
