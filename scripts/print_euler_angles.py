#!/usr/bin/env python

# Just a little Subscriber node that prints out the copter's euler angles

## JSW Jan 2018

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import tf


class PrintEuler(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.degrees = rospy.get_param('~is_in_degrees', True)
        self.topic_str = rospy.get_param('~topic_string', '/mavros_ned_node/estimate')

        # Initialize subscriber
        self.state_sub = rospy.Subscriber(self.topic_str, Odometry, self.state_callback)


    def state_callback(self, msg):

        # get the quaternion orientation from the message
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        # convert to euler angles 
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # update class variables
        self.phi = euler[0]
        self.theta = euler[1]
        self.psi = euler[2]

        if self.degrees:
            print "Euler Angles (degrees):"
            print "Roll: %f" % np.degrees(self.phi)
            print "Pitch: %f" % np.degrees(self.theta)
            print "Yaw: %f" % np.degrees(self.psi)
            print "\n"

        else:
            print "Euler Angles (rad):"
            print "Roll: %f" % self.phi
            print "Pitch: %f" % self.theta
            print "Yaw: %f" % self.psi
            print "\n"


def main():
    # initialize a node
    rospy.init_node('euler_printer')

    # create instance of PrintEuler class
    printer = PrintEuler()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()