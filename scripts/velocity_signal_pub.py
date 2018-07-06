#! /usr/bin/env python

## ROS node that publishes a cyclical pattern of velocity commands to roscopter.
## JSW Jul 2018

import rospy
from std_msgs.msg import Bool
from rosflight_msgs.msg import Command
import numpy as np


class VelPub(object):

    def __init__(self):

        # Load ROS params.
        self.axis = rospy.get_param('~xy_axis', 'X')
        self.magnitude = rospy.get_param('~magnitude', 0.5)
        self.feed_forward = rospy.get_param('~feed_forward', 0.0)

        # Initialize other class variables.
        self.vel_x = 0.0
        self.vel_y = 0.0

        self.phase = 1

        self.t_start = rospy.get_time()

        # Messages
        self.command_msg = Command()
        self.ibvs_active_msg = Bool()
        self.ibvs_active_msg.data = True

        # Initialize publishes
        self.command_pub = rospy.Publisher('/quadcopter/high_level_command', Command, queue_size=5, latch=True)
        self.ibvs_active_pub = rospy.Publisher('/quadcopter/ibvs_active', Bool, queue_size=1)

        self.count_rate = 0.25  # seconds
        self.count_timer = rospy.Timer(rospy.Duration(1.0/self.count_rate), self.generate_velocities)

        # Initialize timers.
        self.update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_commands)


    def send_commands(self, event):

        # Fill out the message.
        self.command_msg.header.stamp = rospy.get_rostime()
        self.command_msg.x = self.vel_x
        self.command_msg.y = self.vel_y
        self.command_msg.F = 0.0
        self.command_msg.z = 0.0
        self.command_msg.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE

        # Publish the message.
        self.ibvs_active_pub.publish(self.ibvs_active_msg)
        self.command_pub.publish(self.command_msg)


    def generate_velocities(self, event):

        # The goal here is to generate a square wave
        now = rospy.get_time()
        if (now - self.t_start) > 5.0:

            # Where are we on the wave pattern?
            if self.phase == 1:
                self.make_wave(self.phase)
                self.phase = 2
            elif self.phase == 2:
                self.make_wave(self.phase)
                self.phase = 3
            elif self.phase == 3:
                self.make_wave(self.phase)
                self.phase = 4
            elif self.phase == 4:
                self.make_wave(self.phase)
                self.phase = 1
            else:
                print 'Unhandeled phase.'
        else:
            pass

    
    def make_wave(self, phase):

        if self.axis == 'X':
            if self.phase == 1:
                self.vel_x = 0.0 + self.feed_forward
            elif self.phase == 2:
                self.vel_x = self.magnitude + self.feed_forward
            elif self.phase == 3:
                self.vel_x = 0.0 + self.feed_forward
            elif self.phase == 4:
                self.vel_x = -self.magnitude + self.feed_forward

        elif self.axis == 'Y':
            if self.phase == 1:
                self.vel_y = 0.0 + self.feed_forward
            elif self.phase == 2:
                self.vel_y = self.magnitude + self.feed_forward
            elif self.phase == 3:
                self.vel_y = 0.0 + self.feed_forward
            elif self.phase == 4:
                self.vel_y = -self.magnitude + self.feed_forward
        else:
            print 'Unhandeled axis.'


        

def main():
    # initialize a node
    rospy.init_node('vel_pub')

    # create instance of VelPub class
    test = VelPub()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
