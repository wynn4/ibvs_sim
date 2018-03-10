#! /usr/bin/env python

## ROS node that publishes a cyclical pattern of thrust commands to PX4 via mavros.
## JSW Jan 2018

import rospy
from mavros_msgs.msg import AttitudeTarget
from geometry_msgs.msg import Vector3
import numpy as np


class TestMAVROSThrust(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.thrust_val = 0.0

        self.count = 0

        # Create a maks of AttitudeTarget msg fields we wish to ignore.
        # Here we have 'OR'ed together all of the fields we don't want
        # (i.e., we want thrust).
        self.ignore_mask = (AttitudeTarget.IGNORE_ROLL_RATE
                            | AttitudeTarget.IGNORE_PITCH_RATE
                            | AttitudeTarget.IGNORE_YAW_RATE
                            | AttitudeTarget.IGNORE_ATTITUDE)
        
        # Initialize timers.
        self.update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_commands)

        self.count_rate = 1.0  # seconds
        self.count_timer = rospy.Timer(rospy.Duration(1.0/self.count_rate), self.generate_thrusts)

        # Initialize publisher
        self.command_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)


    def send_commands(self, event):

        # Create the AttitudeTarget command message.
        command_msg = AttitudeTarget()

        # Fill out the message.
        command_msg.header.stamp = rospy.get_rostime()
        command_msg.type_mask = self.ignore_mask
        command_msg.thrust = self.thrust_val

        # Publish the message.
        self.command_pub.publish(command_msg)


    def generate_thrusts(self, event):

        self.count += 1
        
        # full, zero, half, zero, repeat.

        # full
        if self.count > 0 and self.count <= 3:

            self.thrust_val = 1.0

        # zero
        if self.count > 3 and self.count <= 6:

            self.thrust_val = 0.0

        # half
        if self.count > 6 and self.count <= 9:

            self.thrust_val = 0.5

        # Back to zero for 6 seconds
        if self.count > 9 and self.count <= 15:

            self.thrust_val = 0.0

        # Reset
        if self.count == 16:
            self.count = 0
        

def main():
    # initialize a node
    rospy.init_node('test_mavros_thrust')

    # create instance of TestMAVROSThrust class
    test = TestMAVROSThrust()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
