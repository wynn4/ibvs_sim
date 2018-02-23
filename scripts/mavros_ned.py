#! /usr/bin/env python

## ROS node that takes data from mavros and republishes it the NED frame.
## JSW Feb 2018

import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
import numpy as np
import tf


class MavrosNED(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.px4_estimate_msg = Odometry()

        # vectors to hold data coming from mavros
        self.euler_vec_flu = np.zeros((3,1), dtype=np.float32)
        self.position_vec_enu = np.zeros((3,1), dtype=np.float32)
        self.velocity_vec_lin_flu = np.zeros((3,1), dtype=np.float32)
        self.velocity_vec_ang_flu = np.zeros((3,1), dtype=np.float32)
        

        # Initialize rotation from body FLU to FRD.
        self.R_flu_frd = np.array([[1., 0., 0.],
                                   [0., -1., 0.],
                                   [0., 0., -1.]])

        # Initialize rotation from ENU to NED.
        self.R_enu_ned = np.array([[0., 1., 0.],
                                   [1., 0., 0.],
                                   [0., 0., -1.]])

        # vectors to hold ned data to be published
        self.quaternion_frd = np.array([0, 0, 0, 1], dtype=np.float32)
        self.position_vec_ned = np.zeros((3,1), dtype=np.float32)
        self.velocity_vec_lin_frd = np.zeros((3,1), dtype=np.float32)
        self.velocity_vec_ang_frd = np.zeros((3,1), dtype=np.float32)

        # self.cb_time = rospy.get_time()
        
        # Initialize timers.
        self.update_rate = 29.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_ned_estimate)

        # Initialize subscribers
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pose_callback)
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.mavros_velocity_callback)

        # Initialize publisher
        self.estimate_pub = rospy.Publisher("mavros_ned/estimate", Odometry, queue_size=1)


    def send_ned_estimate(self, event):

        # fill out the message and publish
        self.px4_estimate_msg.header.stamp = rospy.Time.now()
        self.px4_estimate_msg.pose.pose.position.x = self.position_vec_ned[0][0]
        self.px4_estimate_msg.pose.pose.position.y = self.position_vec_ned[1][0]
        self.px4_estimate_msg.pose.pose.position.z = self.position_vec_ned[2][0]

        self.px4_estimate_msg.pose.pose.orientation.x = self.quaternion_frd[0]
        self.px4_estimate_msg.pose.pose.orientation.y = self.quaternion_frd[1]
        self.px4_estimate_msg.pose.pose.orientation.z = self.quaternion_frd[2]
        self.px4_estimate_msg.pose.pose.orientation.w = self.quaternion_frd[3]

        self.px4_estimate_msg.twist.twist.linear.x = self.velocity_vec_lin_frd[0][0]
        self.px4_estimate_msg.twist.twist.linear.y = self.velocity_vec_lin_frd[1][0]
        self.px4_estimate_msg.twist.twist.linear.z = self.velocity_vec_lin_frd[2][0]

        self.px4_estimate_msg.twist.twist.angular.x = self.velocity_vec_ang_frd[0][0]
        self.px4_estimate_msg.twist.twist.angular.y = self.velocity_vec_ang_frd[1][0]
        self.px4_estimate_msg.twist.twist.angular.z = self.velocity_vec_ang_frd[2][0]

        self.estimate_pub.publish(self.px4_estimate_msg)
        # now = rospy.get_time()
        # delay = now - self.cb_time
        # print(delay)


    def mavros_pose_callback(self, msg):

        # self.cb_time = rospy.get_time()
        # then = rospy.get_time()

        # get the position data and populate the vector
        self.position_vec_enu[0][0] = msg.pose.position.x
        self.position_vec_enu[1][0] = msg.pose.position.y
        self.position_vec_enu[2][0] = msg.pose.position.z

        # rotate from enu to ned and store in class variable for use later
        self.position_vec_ned = self.enu_to_ned(self.position_vec_enu)
        
        # get the quaternion orientation from the message
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)

        # convert to euler angles 
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # populate the vector
        self.euler_vec_flu[0][0] = euler[0]
        self.euler_vec_flu[1][0] = euler[1]
        self.euler_vec_flu[2][0] = euler[2]

        # rotate from flu to frd and store in class variable for use later
        euler_vec_frd = self.flu_to_frd(self.euler_vec_flu)

        # convert back to quaternion and store in class variable for use later
        self.quaternion_frd = tf.transformations.quaternion_from_euler(euler_vec_frd[0][0], euler_vec_frd[1][0], euler_vec_frd[2][0])
        # now = rospy.get_time()
        # delay = now - then
        # print(delay)


    def mavros_velocity_callback(self, msg):

        # populate flu linear velocity vector
        self.velocity_vec_lin_flu[0][0] = msg.twist.linear.x
        self.velocity_vec_lin_flu[1][0] = msg.twist.linear.y
        self.velocity_vec_lin_flu[2][0] = msg.twist.linear.z

        # rotate from flu to frd and store in class variable for use later
        self.velocity_vec_lin_frd = self.flu_to_frd(self.velocity_vec_lin_flu)

        # populate flu angular velocity vector
        self.velocity_vec_ang_flu[0][0] = msg.twist.angular.x
        self.velocity_vec_ang_flu[1][0] = msg.twist.angular.y
        self.velocity_vec_ang_flu[2][0] = msg.twist.angular.z

        # rotate from flu to frd and store in class variable for use later
        self.velocity_vec_ang_frd = self.flu_to_frd(self.velocity_vec_ang_flu)


    def enu_to_ned(self, enu_vec):

        ned_vec = np.dot(self.R_enu_ned, enu_vec)
        return ned_vec


    def flu_to_frd(self, flu_vec):

        frd_vec = np.dot(self.R_flu_frd, flu_vec)
        return frd_vec


def main():
    # initialize a node
    rospy.init_node('mavros_ned')

    # create instance of MavrosNED class
    mavros_ned = MavrosNED()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
