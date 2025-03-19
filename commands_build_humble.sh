#!/bin/bash

set -e

cd ~/Robot5A-RaspberryPi
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build --symlink-install --cmake-clean-cache
source install/setup.bash