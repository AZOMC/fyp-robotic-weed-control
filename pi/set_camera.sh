#!/bin/bash
v4l2-ctl -d /dev/video0 --set-ctrl=contrast=140
v4l2-ctl -d /dev/video0 --set-ctrl=saturation=90
v4l2-ctl -d /dev/video0 --set-ctrl=sharpness=100
v4l2-ctl -d /dev/video0 --set-ctrl=backlight_compensation=0
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_automatic=0
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature=140
v4l2-ctl -d /dev/video0 --set-ctrl=brightness=110
v4l2-ctl -d /dev/video0 --set-ctrl=gain=70
