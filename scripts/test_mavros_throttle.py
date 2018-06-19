#! /usr/bin/env python

## ROS node that publishes a cyclical pattern of thrust commands to PX4 via mavros.
## JSW Jan 2018

import rospy
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Vector3
import numpy as np


class TestThrottle(object):

    def __init__(self):

        # Load ROS params.

        heading = rospy.get_param('~heading', 0.0)

        self.heading = np.radians(heading)

        # Create a maks of PositionTarget msg fields we wish to ignore.
        # Here we have 'OR'ed together all of the fields we don't want
        # (i.e., we want x-pos, y-pos, z-pos, yaw).
        self.ignore_mask = (AttitudeTarget.IGNORE_ATTITUDE)

        self.thrust_val = 0.0
        

        # Initialize publisher
        self.command_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)

        # Initialize timers.
        self.update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_commands)

        self.cycle_rate = 1.0
        self.cycle_rate_timer = rospy.Timer(rospy.Duration(1.0/self.cycle_rate), self.cycle)


    def send_commands(self, event):

        command_msg = AttitudeTarget()
        command_msg.header.stamp = rospy.get_rostime()
        command_msg.type_mask = self.ignore_mask

        command_msg.body_rate.x = 0.0
        command_msg.body_rate.y = 0.0
        command_msg.body_rate.z = 0.0

        command_msg.thrust = self.thrust_val

        # Publish.
        self.command_pub.publish(command_msg)


    def cycle(self, event):

        print self.thrust_val

        if self.thrust_val == 1.0:
            self.thrust_val = 0.0
        else:
            self.thrust_val = self.thrust_val + 0.1






def main():
    # initialize a node
    rospy.init_node('test_mavros')

    # create instance of TestThrottle class
    test = TestThrottle()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()