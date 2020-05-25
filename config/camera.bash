#Turn off the auto exposure function of the camera
#corresponding to /dev/video0.
#Adjust the focus and exposure if necessary.
#This script requires v4l2-ctl
#sudo apt install v4l-utils

#!/bin/bash
v4l2-ctl -d /dev/video0 --all
v4l2-ctl -d /dev/video0 --set-ctrl white_balance_temperature_auto=0
v4l2-ctl -d /dev/video0 --set-ctrl white_balance_temperature=4000
#v4l2-ctl -d /dev/video0 --set-ctrl focus_auto=0
#v4l2-ctl -d /dev/video0 --set-ctrl focus_absolute=0
#v4l2-ctl -d /dev/video0 --set-ctrl exposure_auto=1
#v4l2-ctl -d /dev/video0 --set-ctrl exposure_absolute=500
