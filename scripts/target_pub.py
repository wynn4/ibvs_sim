#! /usr/bin/env python

## ROS node that publishes NE location of some target.
## JSW Jan 2018

import rospy
from nav_msgs.msg import Odometry
import numpy as np


class TargetPositionPublisher(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.target_N = rospy.get_param('~target_N', 0.0)
        self.target_E = rospy.get_param('~target_E', 0.0)
        
        # Initialize timers.
        self.update_rate = 1.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_position)

        # Initialize publisher
        self.position_pub = rospy.Publisher("/target_position", Odometry, queue_size=1)


    def update_position(self, event):
        
        target_msg = Odometry()
        target_msg.header.stamp = rospy.Time.now()
        target_msg.pose.pose.position.x = self.target_N
        target_msg.pose.pose.position.y = self.target_E

        self.position_pub.publish(target_msg)


def main():
    # initialize a node
    rospy.init_node('target_pub')

    # create instance of TargetPositionPublisher class
    pos = TargetPositionPublisher()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
