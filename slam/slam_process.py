#!/usr/bin/env python3
"""
slam_process.py - SLAM background process.

This module contains the function that runs in a dedicated child process to
perform SLAM. Running SLAM in a separate process gives it its own Python GIL,
so heavy LIDAR reading and map-building never stalls the terminal UI.

The process reads LIDAR scans, resamples them into fixed-size angle bins, and
feeds them into BreezySLAM's RMHC_SLAM algorithm. The resulting robot pose
and occupancy map are written into a ProcessSharedState object that the UI
process can read at any time.

Scan resampling
---------------
The RPLidar reports measurements at irregular angles. BreezySLAM expects a
fixed array of SCAN_SIZE evenly-spaced readings. The resampling step bins raw
readings by their rounded angle (0-359 degrees) and averages any multiple
readings that fall in the same bin. Bins with no readings are filled with
MAX_DISTANCE_MM (treated as "no obstacle detected").

IMPROVEMENT 2 & 3: after each pose update the robot's position is appended to
the path ring buffer in ProcessSharedState (if it has moved at least
PATH_MIN_DIST_MM since the last recorded point).
"""

from __future__ import annotations

import time
from typing import Optional

from settings import (
    SCAN_SIZE,
    SCAN_RATE_HZ,
    DETECTION_ANGLE,
    MAX_DISTANCE_MM,
    MAP_SIZE_PIXELS,
    MAP_SIZE_METERS,
    HOLE_WIDTH_MM,
    MAP_QUALITY,
    LIDAR_OFFSET_DEG,
    MIN_VALID_POINTS,
    INITIAL_ROUNDS_SKIP,
    MAP_UPDATE_INTERVAL,
    MAX_PATH_POINTS,
    PATH_MIN_DIST_MM,
)
from shared_state import ProcessSharedState

_SCAN_ANGLES: list[float] = [
    float(i * DETECTION_ANGLE / SCAN_SIZE)
    for i in range(SCAN_SIZE)
]


def _resample_scan(
    raw_angles: list[float],
    raw_distances: list[float],
) -> tuple[list[int], int]:
    """Resample raw LIDAR readings into SCAN_SIZE equal-angle bins."""
    bin_sums = [0.0] * SCAN_SIZE
    bin_counts = [0] * SCAN_SIZE

    for angle, dist in zip(raw_angles, raw_distances):
        if dist <= 0:
            continue

        # Convert from RPLidar CW to BreezySLAM CCW convention, then apply offset.
        ccw_angle = -angle + LIDAR_OFFSET_DEG
        bin_idx = int(round(ccw_angle)) % SCAN_SIZE
        bin_sums[bin_idx] += dist
        bin_counts[bin_idx] += 1

    scan_distances: list[int] = []
    valid = 0

    for i in range(SCAN_SIZE):
        if bin_counts[i] > 0:
            avg = bin_sums[i] / bin_counts[i]
            if avg >= MAX_DISTANCE_MM:
                scan_distances.append(MAX_DISTANCE_MM)
            else:
                scan_distances.append(int(avg))
                valid += 1
        else:
            scan_distances.append(MAX_DISTANCE_MM)

    return scan_distances, valid


# ---------------------------------------------------------------------------
# IMPROVEMENT 2 & 3: path recording helper
# ---------------------------------------------------------------------------

_last_path_x: Optional[float] = None
_last_path_y: Optional[float] = None


def _record_path_point(pss: ProcessSharedState, x_mm: float, y_mm: float) -> None:
    """Append (x_mm, y_mm) to the shared path ring buffer if moved enough."""
    global _last_path_x, _last_path_y

    if _last_path_x is not None:
        dx = x_mm - _last_path_x
        dy = y_mm - _last_path_y
        if (dx * dx + dy * dy) < PATH_MIN_DIST_MM ** 2:
            return

    _last_path_x = x_mm
    _last_path_y = y_mm

    head = pss.path_head.value
    pss.path_x[head] = x_mm
    pss.path_y[head] = y_mm

    if pss.path_count.value < MAX_PATH_POINTS:
        pss.path_count.value += 1

    pss.path_head.value = (head + 1) % MAX_PATH_POINTS


def run_slam_process(pss: ProcessSharedState) -> None:
    """Entry point for the SLAM child process."""
    try:
        from breezyslam.algorithms import RMHC_SLAM
        from breezyslam.sensors import Laser
    except ImportError:
        pss.set_error('BreezySLAM not installed. Run: bash install_slam.sh')
        pss.stopped.value = True
        return

    try:
        import lidar as lidar_driver
    except ImportError:
        pss.set_error('lidar.py not found in the slam/ directory')
        pss.stopped.value = True
        return

    lidar = lidar_driver.connect()
    if lidar is None:
        from settings import LIDAR_PORT
        pss.set_error(f'Could not connect to LIDAR on {LIDAR_PORT}')
        pss.stopped.value = True
        return

    scan_mode = lidar_driver.get_scan_mode(lidar)

    laser = Laser(SCAN_SIZE, SCAN_RATE_HZ, DETECTION_ANGLE, MAX_DISTANCE_MM)
    slam = RMHC_SLAM(
        laser,
        MAP_SIZE_PIXELS,
        MAP_SIZE_METERS,
        hole_width_mm=HOLE_WIDTH_MM,
        map_quality=MAP_QUALITY,
    )

    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    pss.set_status(f'connected (mode {scan_mode})')
    pss.connected.value = True

    previous_distances: Optional[list[int]] = None
    round_num = 0
    last_map_update = time.monotonic()

    try:
        for raw_angles, raw_distances in lidar_driver.scan_rounds(lidar, scan_mode):
            if pss.stop_event.is_set():
                break

            round_num += 1
            pss.rounds_seen.value = round_num

            if round_num <= INITIAL_ROUNDS_SKIP:
                pss.valid_points.value = 0
                pss.set_status(f'warming up {round_num}/{INITIAL_ROUNDS_SKIP}')
                continue

            if pss.paused.value:
                pss.set_status('paused')
                continue

            scan_distances, valid = _resample_scan(raw_angles, raw_distances)
            pss.valid_points.value = valid

            if valid >= MIN_VALID_POINTS:
                slam.update(scan_distances, scan_angles_degrees=_SCAN_ANGLES)
                previous_distances = list(scan_distances)
                note = f'live ({valid} pts)'
            elif previous_distances is not None:
                slam.update(previous_distances, scan_angles_degrees=_SCAN_ANGLES)
                note = f'reusing previous ({valid} pts)'
            else:
                pss.set_status(f'waiting ({valid} pts)')
                continue

            x_mm, y_mm, theta_deg = slam.getpos()
            pss.x_mm.value = x_mm
            pss.y_mm.value = y_mm
            pss.theta_deg.value = theta_deg
            pss.pose_version.value += 1

            _record_path_point(pss, x_mm, y_mm)

            now = time.monotonic()
            if now - last_map_update >= MAP_UPDATE_INTERVAL:
                slam.getmap(mapbytes)
                pss.shm.buf[:len(mapbytes)] = mapbytes
                pss.map_version.value += 1
                last_map_update = now

            pss.set_status(note)

    except Exception as exc:
        pss.set_error(f'SLAM process error: {exc}')
    finally:
        try:
            lidar_driver.disconnect(lidar)
        except Exception:
            pass

        pss.connected.value = False
        pss.stopped.value = True
