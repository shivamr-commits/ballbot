#!/bin/bash
# Usage: ./cam_setup.sh [device]   (default /dev/video0)
D=${1:-/dev/video0}

v4l2-ctl -d $D --set-ctrl=exposure_dynamic_framerate=0
v4l2-ctl -d $D --set-ctrl=auto_exposure=1          # manual mode (must come first)
v4l2-ctl -d $D --set-ctrl=exposure_time_absolute=150
v4l2-ctl -d $D --set-ctrl=gain=100
v4l2-ctl -d $D --set-ctrl=white_balance_automatic=0
v4l2-ctl -d $D --set-ctrl=white_balance_temperature=4000

echo "Applied. Current values:"
v4l2-ctl -d $D --get-ctrl=auto_exposure,exposure_time_absolute,exposure_dynamic_framerate,gain,white_balance_automatic
