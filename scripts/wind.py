#! /usr/bin/env python

## ROS node that publishes a cyclical pattern of velocity commands to PX4 via mavros.
## JSW Jan 2018

import rospy
from geometry_msgs.msg import Vector3
import numpy as np


class Wind(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.wind_x = 5.0
        self.wind_y = 5.0
        self.wind_z = 0.0
        
        # Initialize timers.
        self.update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_wind)

        # Initialize publisher
        self.wind_pub = rospy.Publisher("/quadcopter/wind", Vector3, queue_size=1)


    def update_wind(self, event):
        
        wind_msg = Vector3()
        wind_msg.x = self.wind_x
        wind_msg.y = self.wind_y
        wind_msg.z = self.wind_z

        self.wind_pub.publish(wind_msg)


    


def main():
    # initialize a node
    rospy.init_node('wind_publisher')

    # create instance of Wind class
    wind = Wind()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
