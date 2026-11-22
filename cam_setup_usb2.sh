#!/bin/bash
# Setup for the "USB 2.0 Camera" (Microdia 0c45:62f1).
# This camera has NO exposure/gain/white-balance controls: lighting decides the frame rate.
# Settings reset whenever the camera is unplugged: run this after every replug.
# Usage: bash cam_setup_usb2.sh [device]   (default /dev/video0)
D=${1:-/dev/video0}

v4l2-ctl -d $D --set-ctrl=backlight_compensation=0
v4l2-ctl -d $D --set-ctrl=power_line_frequency=1     # 50 Hz mains (India)

echo "Applied. Current values:"
v4l2-ctl -d $D --get-ctrl=backlight_compensation,power_line_frequency,brightness,contrast,saturation
