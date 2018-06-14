#! /usr/bin/env python

## ROS node that re-publishes NE Velocity and Latitude and Longitude from the Intertial Sense.
## JSW Jan 2018

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import Point32
import numpy as np


class BoatVelocityPublisher(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.vx = 0.0
        self.vy = 0.0
        self.vn = 0.0
        self.ve = 0.0

        self.psi = 0.0

        # self.lat = 0.0
        # self.lon = 0.0

        # Initialize messages
        self.velocity_msg = Point32()
        # self.lat_lon_msg = Point32()
        
        # Initialize timers.
        # self.update_rate = 1.0
        # self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_position)

        # Initialize publishers
        self.velocity_pub = rospy.Publisher('/boat_ne_velocity', Point32, queue_size=1)
        # self.lat_lon_pub = rospy.Publisher('/ins_lat_lon', Point32, queue_size=1)

        # Initialize subscriber
        self.boat_odom_sub = rospy.Subscriber('/boat/odometry/NED', Odometry, self.boat_odom_callback)
        self.boat_euler_sub = rospy.Subscriber('/boat/euler', Vector3Stamped, self.boat_euler_callback)

        # Initialize timers.
        self.update_rate = 5.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_data)


    def send_data(self, event):

        # Rotate velocities to be in the vehicle frame (NED aligned)
        self.vn = self.vx*np.cos(self.psi) - self.vy*np.sin(self.psi)
        self.ve = self.vy*np.cos(self.psi) + self.vx*np.sin(self.psi)

        # Fill out the messages.
        self.velocity_msg.x = self.vn
        self.velocity_msg.y = self.ve

        # Publish.
        self.velocity_pub.publish(self.velocity_msg)

        # print "vn: %f" % self.vn
        # print "ve: %f" % self.ve


    def boat_odom_callback(self, msg):

        # Grab the body-frame velocities
        self.vx = msg.twist.twist.linear.x
        self.vy = msg.twist.twist.linear.y


    def boat_euler_callback(self, msg):

        # Just grab the heading angle
        self.psi = msg.vector.z
        

def main():
    # initialize a node
    rospy.init_node('boat_vel_pub')

    # create instance of BoatVelocityPublisher class
    boat_vel_pub = BoatVelocityPublisher()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
