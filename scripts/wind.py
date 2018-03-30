#! /usr/bin/env python

## ROS node that publishes wind.
## JSW Jan 2018

import rospy
from geometry_msgs.msg import Vector3
import numpy as np


class Wind(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.wind_N = rospy.get_param('~wind_N', 0.0)
        self.wind_E = rospy.get_param('~wind_E', 0.0)
        self.wind_D = rospy.get_param('~wind_D', 0.0)
        
        # Initialize timers.
        self.update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_wind)

        # Initialize publisher
        self.wind_pub = rospy.Publisher("/quadcopter/wind", Vector3, queue_size=1)


    def update_wind(self, event):
        
        wind_msg = Vector3()
        wind_msg.x = self.wind_N
        wind_msg.y = self.wind_E
        wind_msg.z = self.wind_D

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
