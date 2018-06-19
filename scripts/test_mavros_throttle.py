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

        self.thrust_val = 0.5
        self.Time = 0.0
        
        self.landing_thrust0 = 0.45
        self.landed = False

        # Initialize publisher
        # self.other_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.land_pub = rospy.Publisher("/mavros/setpoint_raw/attitude", AttitudeTarget, queue_size=1)

        # Initialize timers.
        self.update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_commands)

        self.cycle_rate = 1.0
        self.cycle_rate_timer = rospy.Timer(rospy.Duration(1.0/self.cycle_rate), self.cycle)


    # def send_commands(self, event):

    #     waypoint_command_msg = PositionTarget()
    #     waypoint_command_msg.header.stamp = rospy.get_rostime()
    #     waypoint_command_msg.coordinate_frame = waypoint_command_msg.FRAME_LOCAL_NED
    #     waypoint_command_msg.type_mask = self.ignore_mask

    #     waypoint_command_msg.position.x = 0.0  # E
    #     waypoint_command_msg.position.y = 0.0  # N
    #     waypoint_command_msg.position.z = 15.0  # U

    #     yaw = -self.heading + np.radians(90.0)
    #     while yaw > np.radians(180.0):  yaw = yaw - np.radians(360.0)
    #     while yaw < np.radians(-180.0):  yaw = yaw + np.radians(360.0)

    #     waypoint_command_msg.yaw = yaw

    #     # Publish.
    #     self.command_pub.publish(waypoint_command_msg)


    def send_commands(self, event):

        command_msg = AttitudeTarget()
        command_msg.header.stamp = rospy.get_rostime()
        command_msg.type_mask = self.ignore_mask

        command_msg.body_rate.x = 0.0
        command_msg.body_rate.y = 0.0
        command_msg.body_rate.z = 0.0

        command_msg.thrust = self.thrust_val
        print self.thrust_val

        # Publish.
        self.land_pub.publish(command_msg)


    def cycle(self, event):

        # Increment.
        self.Time = self.Time + 1.0
        print self.Time

        if self.Time >= 5.0 and self.landed == False:
            self.do_landing(self.landing_thrust0)
        else:
            pass


    def do_landing(self, start_thrust):

        self.thrust_val = start_thrust

        # We want the ramp down to take place in 1.0 seconds
        ramp_time = 0.5
        decrement = start_thrust / 10.0


        for x in range(0, 10):

            self.thrust_val -= decrement
            print self.thrust_val
            rospy.sleep(ramp_time / 10.0)

        # Disarm Here
        print "Disarm."
        self.landed = True








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