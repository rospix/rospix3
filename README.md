# Rospix3

| Build status | [![Build Status](https://github.com/rospix/rospix3/workflows/Melodic/badge.svg)](https://github.com/rospix/rospix3/actions) | [![Build Status](https://github.com/rospix/rospix3/workflows/Noetic/badge.svg)](https://github.com/rospix/rospix3/actions) |
|--------------|-----------------------------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------------|

ROS interface for TPX3 devices on Advacam hardware interface.

## Compatibility

ROS Melodic / Noetic

## Dependencies

ROS, catkin-tools, and other dependencies required for compiling Rospix3 with its other dependencies:
```bash
./installation/install_dependencies.sh
```

Clone the following packages into your workspace and compile together with **Rospix3**.

* [mrs_lib](https://github.com/ctu-mrs/mrs_lib)
  * [mrs_msgs](https://github.com/ctu-mrs/mrs_msgs)
* [rad_msgs](https://github.com/rospix/rad_msgs)
