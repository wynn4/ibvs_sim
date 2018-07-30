#! /usr/bin/env python

## ROS node that receives GPS data from the landing target and converts it to NED for the copter.
## JSW Jan 2018

import rospy
from mavros_msgs.msg import State
# from mavros_msgs.msg import HomePosition
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
import numpy as np


class TargetToNED(object):

    def __init__(self):

        # Load ROS params.

        self.copter_is_armed = False
        self.ready_to_publish = False

        # Default to the intersection in Springville
        self.home_lat = 40.174346
        self.home_lon = -111.651892

        # Radius of the earth in Springville UT
        self.R_earth = 6370651.0

        # Other
        self.target_north = 0.0
        self.target_east = 0.0

        # GPS biases
        self.bias_north = rospy.get_param('~bias_north', 0.0)
        self.bias_east = rospy.get_param('~bias_east', 0.0)

        self.target_msg = Odometry()

        # Initialize publishers
        self.target_ned_pub = rospy.Publisher('/target_position', Odometry, queue_size=1)

        # Initialize subscribers.
        self.status_sub = rospy.Subscriber('/mavros/state', State, self.mavros_status_callback)
        self.gp_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.mavros_gp_callback)

        self.target_sub = rospy.Subscriber('/ins_lat_lon', Point, self.target_callback)


    def target_callback(self, msg):

        # Pull off the data.
        latitude = msg.x
        longitude = msg.y

        # Convert from LL to NE
        self.target_north = self.R_earth * (latitude - self.home_lat) * np.pi/180.0
        self.target_east = self.R_earth * np.cos(self.home_lat * np.pi/180.0) * (longitude - self.home_lon) * np.pi/180.0

        # Fill out the message.
        self.target_msg.header.stamp = rospy.Time.now()
        self.target_msg.pose.pose.position.x = self.target_north - self.bias_north
        self.target_msg.pose.pose.position.y = self.target_east - self.bias_east

        # Publish.
        if self.ready_to_publish:
            self.target_ned_pub.publish(self.target_msg)
        else:
            pass


    def mavros_status_callback(self, msg):

        # See if the copter is armed.
        if msg.armed:
            self.copter_is_armed = True
        else:
            pass


    def mavros_gp_callback(self, msg):

        # Data on this topic is the filtered and fused global position estimate
        # I would prefer to use data from the '/mavros/global_position/home' topic,
        # but it is not being published for some reason.

        lat = msg.latitude
        lon = msg.longitude

        if self.copter_is_armed:

            # Store our home latitude and longitude.
            self.home_lat = lat
            self.home_lon = lon

            self.ready_to_publish = True

            print "Target_to_NED: Home Location Set."

            # Unregister the subscribers
            self.status_sub.unregister()
            self.gp_sub.unregister()

        else:
            pass


def main():
    # initialize a node
    rospy.init_node('target_to_ned')

    # create instance of TargetToNED class
    converter = TargetToNED()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
