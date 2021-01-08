#!/bin/bash
set -e

MY_PATH=`pwd`

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

echo "Creating workspace"
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
echo "Cloning dependencies"
git clone https://github.com/ctu-mrs/mrs_msgs.git
git clone https://github.com/ctu-mrs/mrs_lib.git
git clone https://github.com/ctu-mrs/rad_msgs.git
ln -s $MY_PATH ./
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/catkin_ws

echo "Starting build"
cd ~/catkin_ws
source /opt/ros/$ROS_DISTRO/setup.bash
catkin build --limit-status-rate 0.2 --summarize
echo "Ended build"
