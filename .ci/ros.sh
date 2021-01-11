#!/bin/bash

set -e

distro=`lsb_release -r | awk '{ print $2 }'`
[ "$distro" = "18.04" ] && ROS_DISTRO="melodic"
[ "$distro" = "20.04" ] && ROS_DISTRO="noetic"

trap 'last_command=$current_command; current_command=$BASH_COMMAND' DEBUG
trap 'echo "$0: \"${last_command}\" command failed with exit code $?"' ERR

distro=`lsb_release -r | awk '{ print $2 }'`

echo "$0: Installing ROS"

sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

for server in ha.pool.sks-keyservers.net \
              hkp://p80.pool.sks-keyservers.net:80 \
              keyserver.ubuntu.com \
              hkp://keyserver.ubuntu.com:80 \
              pgp.mit.edu; do
    sudo apt-key adv --keyserver "$server" --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && break || echo "Trying new server..."
done

sudo apt-get -y update

[ "$distro" = "18.04" ] && sudo apt-get -y install ros-melodic-ros-base
[ "$distro" = "20.04" ] && sudo apt-get -y install ros-noetic-ros-base

num=`cat ~/.bashrc | grep "/opt/ros/$ROS_DISTRO/setup.bash" | wc -l`
if [ "$num" -lt "1" ]; then

  # set bashrc
  echo "
source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

fi

# general ros-related
sudo apt-get -y install \
  ros-$ROS_DISTRO-tf-conversions\
  ros-$ROS_DISTRO-tf2-sensor-msgs\
  ros-$ROS_DISTRO-tf2-geometry-msgs\
  ros-$ROS_DISTRO-tf2-eigen\
  ros-$ROS_DISTRO-cv-bridge\
  ros-$ROS_DISTRO-image-transport\
  ros-$ROS_DISTRO-image-transport-plugins\
  ros-$ROS_DISTRO-compressed-image-transport\
  ros-$ROS_DISTRO-theora-image-transport\
  ros-$ROS_DISTRO-pcl-ros\
  ros-$ROS_DISTRO-pcl-conversions\

if [ "$distro" = "18.04" ]; then

sudo apt-get -y install \
  python-setuptools\
  python3-setuptools\
  python-catkin-tools\

elif [ "$distro" = "20.04" ]; then

sudo apt-get -y install \
  python3-setuptools\
  python3-catkin-tools\
  python3-osrf-pycommon\

fi

# other
sudo apt-get -y install \
  libopencv-dev\
