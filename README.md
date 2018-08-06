IBVS Sim ROS Package
====================

This package contains source files needed to simulate multirotor precision landing on stationary ground targets, and moving targets exhibiting ship-like motion. The simulation environment is ROS/Gazebo. Also included in this repository are launch files for performing precision landings in hardware using a PX4-enabled multirotor.

## Workspace Setup ##

To run simulations, you will need to clone this repository into the `/src` folder of your catkin workspace along with the following additional packages:

* [ROScopter](https://github.com/byu-magicc/roscopter/commits/master)
* [ROSflight](https://github.com/rosflight/rosflight)
* [ROSflight Plugins](https://github.com/byu-magicc/rosflight_plugins)
* [MAGICC SIM](https://github.com/byu-magicc/magicc_sim)
* [ArUco Localization](https://github.com/wynn4/aruco_localization)
