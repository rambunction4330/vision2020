#!/bin/sh
v4l2-ctl --device 0 --set-ctrl brightness=0
v4l2-ctl --device 0 --set-ctrl contrast=255
v4l2-ctl --device 0 --set-ctrl saturation=255
v4l2-ctl --device 0 --set-ctrl gain=0
v4l2-ctl --device 0 --set-ctrl sharpness=0
v4l2-ctl --device 0 --set-ctrl exposure_auto=1
v4l2-ctl --device 0 --set-ctrl exposure_absolute=60
v4l2-ctl --device 0 --set-ctrl white_balance_temperature_auto=0
v4l2-ctl --device 0 --set-ctrl white_balance_temperature=6500
