#! /usr/bin/env python

## JSW Jul 2018

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import numpy as np

class BiasPrinter(object):

    def __init__(self):

        # Load ROS params.

        # Default to the intersection in Springville
        self.home_lat = 40.174346
        self.home_lon = -111.651892

        # Radius of the earth in Springville UT
        self.R_earth = 6370651.0

        # Initialize publishers

        # Initialize subscribers.
        self.gp_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.mavros_gp_callback)

        self.target_sub = rospy.Subscriber('/ins_lat_lon', Point, self.target_callback)


    def target_callback(self, msg):

        # Pull off the data.
        latitude = msg.x
        longitude = msg.y

        # Convert from LL to NE
        bias_north = self.R_earth * (latitude - self.home_lat) * np.pi/180.0
        bias_east = self.R_earth * np.cos(self.home_lat * np.pi/180.0) * (longitude - self.home_lon) * np.pi/180.0

        print 'bias north: %f' % bias_north
        print 'bias_east: %f' % bias_east
        print '\n'


    def mavros_gp_callback(self, msg):

        self.home_lat = msg.latitude
        self.home_lon = msg.longitude


def main():
    # initialize a node
    rospy.init_node('bias_printer')

    # create instance of BiasPrinter class
    printer = BiasPrinter()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()