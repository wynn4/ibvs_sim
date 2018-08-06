IBVS Sim ROS Package
====================

This package contains source files needed to simulate multirotor precision landings using image-based visual servoing. Simulations for landing on stationary ground targets, and moving targets exhibiting ship-like motion are included. The simulation environment is ROS/Gazebo. Also included in this repository are launch files for performing precision landings in hardware using a PX4-enabled multirotor. Use of this package assumes you have installed Ubuntu 16.04 LTS, ROS Kinetic, and Gazebo.

## Workspace Setup ##

To run simulations, you will need to clone this repository into the `/src` folder of your catkin workspace along with the following additional packages:

* [ROScopter](https://github.com/byu-magicc/roscopter/commits/master)
* [ROSflight](https://github.com/rosflight/rosflight)
* [ROSflight Plugins](https://github.com/byu-magicc/rosflight_plugins)
* [MAGICC SIM](https://github.com/byu-magicc/magicc_sim)
* [ArUco Localization](https://github.com/wynn4/aruco_localization)

You also need to install MAVROS: `sudo apt install ros-kinetic-mavros`


## Landing on a Stationary Ground Target ##
To simulate a multirotor lan
