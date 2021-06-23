#!/bin/bash
# A bash script to run the motion planner and plot the result
stored_date=$(date "+%F_%T")
mkdir data/"$stored_date"
# shellcheck disable=SC2164
cd src/cmake-build-debug
make
cd ../..
src/cmake-build-debug/bt_robotics "$stored_date" "2"
python3 visualization/plot2d.py "$stored_date"

