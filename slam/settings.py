#!/usr/bin/env python3
"""
settings.py - All user-configurable settings for the SLAM system.
Based on the original Studio 16 settings.py.
"""

# ===========================================================================
# LIDAR hardware
# ===========================================================================
LIDAR_PORT = '/dev/ttyUSB0'
LIDAR_BAUD = 115200

# ===========================================================================
# SLAM map
# ===========================================================================
MAP_SIZE_PIXELS = 1000
MAP_SIZE_METERS = 8
MAP_QUALITY     = 5
HOLE_WIDTH_MM   = 100

# ===========================================================================
# Scan settings
# ===========================================================================
SCAN_SIZE        = 360
SCAN_RATE_HZ     = 5
DETECTION_ANGLE  = 360
MAX_DISTANCE_MM  = 12000

# ===========================================================================
# LIDAR mounting offset
# ===========================================================================
# Set to the CCW angle from robot forward to LIDAR forward.
# 0 = LIDAR forward matches robot forward (default).
LIDAR_OFFSET_DEG = 90

# ===========================================================================
# Scan quality thresholds
# ===========================================================================
MIN_VALID_POINTS    = 150
INITIAL_ROUNDS_SKIP = 5

# ===========================================================================
# UI and rendering
# ===========================================================================
UI_REFRESH_HZ       = 4
MAX_RENDER_COLS     = 120
MAX_RENDER_ROWS     = 45
MAP_UPDATE_HZ       = 2.0
MAP_UPDATE_INTERVAL = 1.0 / MAP_UPDATE_HZ
DEFAULT_ZOOM        = 0
PAN_STEP_FRACTION   = 0.20
UNKNOWN_BYTE        = 127

ZOOM_HALF_M = [
    None,
    MAP_SIZE_METERS / 2.0,
    MAP_SIZE_METERS / 3.0,
    MAP_SIZE_METERS / 5.0,
    MAP_SIZE_METERS / 8.0,
]

# ===========================================================================
# Path tracking
# ===========================================================================
MAX_PATH_POINTS  = 2000
PATH_MIN_DIST_MM = 50.0
