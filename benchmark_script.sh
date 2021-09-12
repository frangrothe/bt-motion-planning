#!/bin/bash
# A bash script to run the benchmark and save the log files

# shellcheck disable=SC2164
cd src/cmake-build-debug
make
cd ../..
#src/cmake-build-debug/bt_robotics "testing" "7" "spacetime" "4"
src/cmake-build-debug/bt_robotics "testing" "7" "rrt" "1"
src/cmake-build-debug/bt_robotics "testing" "7" "rrt" "2"
src/cmake-build-debug/bt_robotics "testing" "7" "rrt" "4"
