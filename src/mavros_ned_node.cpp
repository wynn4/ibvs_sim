#include <ros/ros.h>

#include "mavros_ned/MavrosNED.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "mavros_ned_node");
  mavros_ned::MavrosNED thing;

  ros::spin();
  return 0;
}
