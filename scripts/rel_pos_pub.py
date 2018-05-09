#! /usr/bin/env python

## ROS node that publishes relative position between the copter and target (in the copter's coord frame).
## JSW May 2018

import rospy
from nav_msgs.msg import Odometry
import numpy as np


class RelativePositionPublisher(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.copter_N = 0.0
        self.copter_E = 0.0
        self.copter_D = 0.0

        self.target_N = 0.0
        self.target_E = 0.0
        self.target_D = 0.0

        self.vec_msg = Odometry()

        # Initialize publisher.
        self.rel_pos_pub = rospy.Publisher('/relative_position', Odometry, queue_size=1)

        # Initialize subscribers.
        self.copter_sub = rospy.Subscriber('/quadcopter/ground_truth/odometry/NED', Odometry, self.copter_callback)
        self.target_sub = rospy.Subscriber('/target_position', Odometry, self.target_callback)


    def copter_callback(self, msg):

    	self.copter_N = msg.pose.pose.position.x
    	self.copter_E = msg.pose.pose.position.y
    	self.copter_D = msg.pose.pose.position.z

    	# Define vector from copter to the target.
    	vec = np.array([[self.target_N - self.copter_N],
    		            [self.target_E - self.copter_E],
    		            [self.target_D - self.copter_D]])

    	# Define vector from target to the copter.
    	vec = -vec

    	# Publish
    	self.vec_msg.header.stamp = rospy.get_rostime()
    	self.vec_msg.pose.pose.position.x = vec[0][0]
    	self.vec_msg.pose.pose.position.y = vec[1][0]
    	self.vec_msg.pose.pose.position.z = vec[2][0]

    	self.rel_pos_pub.publish(self.vec_msg)




    def target_callback(self, msg):

    	self.target_N = msg.pose.pose.position.x
    	self.target_E = msg.pose.pose.position.y
    	self.target_D = msg.pose.pose.position.z




def main():

    # Initialize a node.
    rospy.init_node('rel_pos_pub')

    # Create instance of RelativePositionPublisher class.
    rel_pos = RelativePositionPublisher()

    # Spin.
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()