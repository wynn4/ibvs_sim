#! /usr/bin/env python

## ROS node that re-publishes NE Velocity and Latitude and Longitude from the Intertial Sense.
## JSW Jan 2018

import rospy
from inertial_sense.msg import GPS
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
import numpy as np


class InertialSenseGPSPublisher(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.vn = 0.0
        self.ve = 0.0

        self.lat = 0.0
        self.lon = 0.0

        # Initialize messages
        self.velocity_msg = Point32()
        self.lat_lon_msg = Point()
        
        # Initialize timers.
        # self.update_rate = 1.0
        # self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_position)

        # Initialize publishers
        self.velocity_pub = rospy.Publisher('/ins_ne_velocity', Point32, queue_size=1)
        self.lat_lon_pub = rospy.Publisher('/ins_lat_lon', Point, queue_size=1)

        # Initialize subscriber
        self.ins_sub = rospy.Subscriber('/gps', GPS, self.gps_callback)


    def gps_callback(self, msg):

        # Pull off the data.
        self.vn = msg.linear_velocity.x
        self.ve = msg.linear_velocity.y

        self.lat = msg.latitude
        self.lon = msg.longitude

        # Fill out the messages.
        self.velocity_msg.x = self.vn
        self.velocity_msg.y = self.ve

        self.lat_lon_msg.x = self.lat
        self.lat_lon_msg.y = self.lon

        # Publish.
        self.velocity_pub.publish(self.velocity_msg)
        self.lat_lon_pub.publish(self.lat_lon_msg)

        # print "vn: %f" % self.vn
        # print "ve: %f" % self.ve
        # print "lat: %f" % self.lat
        # print "lon: %f" % self.lon
        

def main():
    # initialize a node
    rospy.init_node('ins_gps_pub')

    # create instance of InertialSenseGPSPublisher class
    gps_pub = InertialSenseGPSPublisher()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
