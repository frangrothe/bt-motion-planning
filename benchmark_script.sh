#!/bin/bash
# A bash script to run the benchmark and save the log files

# shellcheck disable=SC2164
cd src/cmake-build-debug
make
cd ../..
src/cmake-build-debug/bt_robotics "testing" "7" "spacetime" "-1"
src/cmake-build-debug/bt_robotics "testing" "7" "rrt" "8"
src/cmake-build-debug/bt_robotics "testing" "7" "rrt" "32"
src/cmake-build-debug/bt_robotics "testing" "7" "rrt" "128"
