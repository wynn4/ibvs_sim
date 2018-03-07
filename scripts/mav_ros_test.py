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

        # Initialize other class variables.
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_z = 0.0
        self.yaw_rate = 0.0

        self.count = 0

        # Initialize rotation from NED to ENU (for MAVROS).
        self.R_ned_enu = np.array([[0., 1., 0.],
                                  [1., 0., 0.],
                                  [0., 0., -1.]])

        # Create a maks of PositionTarget msg fields we wish to ignore.
        # Here we have 'OR'ed together all of the fields we don't want
        # (i.e., we want x-vel, y-vel, z-vel, yawrate).
        self.ignore_mask = (PositionTarget.IGNORE_PX
                            | PositionTarget.IGNORE_PY
                            | PositionTarget.IGNORE_PZ
                            | PositionTarget.IGNORE_AFX
                            | PositionTarget.IGNORE_AFY
                            | PositionTarget.IGNORE_AFZ
                            | PositionTarget.FORCE
                            | PositionTarget.IGNORE_YAW)
        
        # Initialize timers.
        self.update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_commands)

        self.count_rate = 1.0  # seconds
        self.count_timer = rospy.Timer(rospy.Duration(1.0/self.count_rate), self.generate_velocities)

        # Initialize publisher
        self.command_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)


    def send_commands(self, event):

        # Create the NED velocity vector.
        ned_vec = np.array([[self.vel_x],
                            [self.vel_y],
                            [self.vel_z]], dtype=np.float32)

        # Rotate the NED vector to ENU (for MAVROS).
        enu_vec = self.ned_to_enu(ned_vec)


        # Create the PositionTarget command message.
        command_msg = PositionTarget()

        # Fill out the message.
        command_msg.header.stamp = rospy.get_rostime()
        # FRAME_BODY_NED is the vehicle-1 frame
        command_msg.coordinate_frame = command_msg.FRAME_BODY_NED
        command_msg.type_mask = self.ignore_mask

        command_msg.velocity.x = enu_vec[0][0]
        command_msg.velocity.y = enu_vec[1][0]
        command_msg.velocity.z = enu_vec[2][0]

        command_msg.yaw_rate = -self.yaw_rate

        # Publish the message.
        self.command_pub.publish(command_msg)


    def generate_velocities(self, event):

        self.count += 1
        
        # Fly a square (Right, Forward, Left, Back).

        # Right
        if self.count > 0 and self.count <= 5:

            self.vel_x = 0.0
            self.vel_y = 0.5
            self.vel_z = 0.0

        # Forward
        if self.count > 5 and self.count <= 10:

            self.vel_x = 0.5
            self.vel_y = 0.0
            self.vel_z = 0.0

        # Left
        if self.count > 10 and self.count <= 15:

            self.vel_x = 0.0
            self.vel_y = -0.5
            self.vel_z = 0.0

        # Back
        if self.count > 15 and self.count <= 20:

            self.vel_x = -0.5
            self.vel_y = 0.0
            self.vel_z = 0.0

        
        # Fly Up then Down.

        # Up
        if self.count > 20 and self.count <= 25:

            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_z = -0.5

        # Down
        if self.count > 25 and self.count <= 30:

            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_z = 0.5


        # Yaw CW then CCW.

        # CW
        if self.count > 30 and self.count <= 35:

            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_z = 0.0
            self.yaw_rate = np.radians(45.0)

        # CCW
        if self.count > 35 and self.count <= 40:

            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_z = 0.0
            self.yaw_rate = np.radians(-45.0)
        
        
        # Sit still for 10 counts.
        if self.count > 40 and self.count <= 50:

            self.vel_x = 0.0
            self.vel_y = 0.0
            self.vel_z = 0.0
            self.yaw_rate = 0.0
        
        
        # Reset
        if self.count == 51:
            self.count = 0


    def ned_to_enu(self, ned_vec):

        enu_vec = np.dot(self.R_ned_enu, ned_vec)
        return enu_vec

        



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
