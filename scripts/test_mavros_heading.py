#! /usr/bin/env python

## ROS node that publishes a cyclical pattern of velocity commands to PX4 via mavros.
## JSW Jan 2018

import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import Vector3
import numpy as np


class TestMAVROS(object):

    def __init__(self):

        # Load ROS params.

        heading = rospy.get_param('~heading', 0.0)

        self.heading = np.radians(heading)

        # Create a maks of PositionTarget msg fields we wish to ignore.
        # Here we have 'OR'ed together all of the fields we don't want
        # (i.e., we want x-pos, y-pos, z-pos, yaw).
        self.ignore_mask = (PositionTarget.IGNORE_VX
                               | PositionTarget.IGNORE_VY
                               | PositionTarget.IGNORE_VZ
                               | PositionTarget.IGNORE_AFX
                               | PositionTarget.IGNORE_AFY
                               | PositionTarget.IGNORE_AFZ
                               | PositionTarget.FORCE
                               | PositionTarget.IGNORE_YAW_RATE)
        

        # Initialize publisher
        self.command_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)

        # Initialize timers.
        self.update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_commands)


    def send_commands(self, event):

        waypoint_command_msg = PositionTarget()
        waypoint_command_msg.header.stamp = rospy.get_rostime()
        waypoint_command_msg.coordinate_frame = waypoint_command_msg.FRAME_LOCAL_NED
        waypoint_command_msg.type_mask = self.ignore_mask

        waypoint_command_msg.position.x = 0.0  # E
        waypoint_command_msg.position.y = 0.0  # N
        waypoint_command_msg.position.z = 15.0  # U

        yaw = -self.heading + np.radians(90.0)
        while yaw > np.radians(180.0):  yaw = yaw - np.radians(360.0)
        while yaw < np.radians(-180.0):  yaw = yaw + np.radians(360.0)

        waypoint_command_msg.yaw = yaw

        # Publish.
        self.command_pub.publish(waypoint_command_msg)


def main():
    # initialize a node
    rospy.init_node('test_mavros')

    # create instance of TestMAVROS class
    test = TestMAVROS()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
