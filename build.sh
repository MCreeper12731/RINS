#! /bin/bash

cd ROS2

rm -rf build
rm -rf log
rm -rf install

colcon build --packages-select $1 --symlink-install

cd ..