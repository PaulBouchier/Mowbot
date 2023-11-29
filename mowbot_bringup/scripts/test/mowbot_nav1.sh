#!/bin/bash
rosrun mowbot_bringup runner.py \
nav 0 1 3 1 -1 -3 \
stop 5 \
movo -2 \
nav 0 0 \
stop
