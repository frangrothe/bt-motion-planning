#!/bin/bash
# A bash script to run the motion planner and plot the result
stored_date=$(date "+%F_%T")
echo "$stored_date"
# shellcheck disable=SC2164
cd src/cmake-build-debug
make
cd ../..
src/cmake-build-debug/bt_robotics "$stored_date"
python3 visualization/plot.py "$stored_date"

