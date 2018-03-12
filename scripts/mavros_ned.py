#! /usr/bin/env python

## ROS node that takes data from mavros and republishes it the NED frame.
## JSW Feb 2018

import rospy
from mavros_msgs.msg import PositionTarget
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
import numpy as np
import tf

# Coordinate Frame Explanations
# Mavros takes linear data (positions and linear velocities) and transforms them to be w.r.t. the ENU coordinate frame.
# For positions, this frame is world-fixed.  For velocities, this frame is body fixed with ENU -> right-front-up

# Mavros takes angular data (orientation and angular velocities) and transforms them to be w.r.t. a FLU coordinate frame.
# For both orientation and angular velocities, this frame is body fixed with FLU -> front-left-up


class MavrosNED(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.px4_estimate_msg = Odometry()

        # Create arrays to hold data coming from mavros.
        self.euler_vec_flu = np.zeros((3,1), dtype=np.float32)
        self.position_vec_enu = np.zeros((3,1), dtype=np.float32)
        self.velocity_vec_lin_rfu = np.zeros((3,1), dtype=np.float32)
        self.velocity_vec_ang_flu = np.zeros((3,1), dtype=np.float32)
        

        # Initialize rotation from body FLU to FRD.
        self.R_flu_frd = np.array([[1, 0, 0],
                                   [0, -1, 0],
                                   [0, 0, -1]], dtype=np.float32)

        # Initialize rotation from ENU to NED.
        self.R_enu_ned = np.array([[0, 1, 0],
                                   [1, 0, 0],
                                   [0, 0, -1]], dtype=np.float32)

        # Create arrays to hold NED/FRD data to be published
        self.quaternion_frd = np.array([0, 0, 0, 1], dtype=np.float32)
        self.position_vec_ned = np.zeros((3,1), dtype=np.float32)
        self.velocity_vec_lin_frd = np.zeros((3,1), dtype=np.float32)
        self.velocity_vec_ang_frd = np.zeros((3,1), dtype=np.float32)

        # self.cb_time = rospy.get_time()
        
        # Initialize timers.
        self.update_rate = 29.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_ned_estimate)

        # Initialize subscribers.
        self.pose_sub = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.mavros_pose_callback)
        self.vel_sub = rospy.Subscriber('/mavros/local_position/velocity', TwistStamped, self.mavros_velocity_callback)

        # Initialize publisher.
        self.estimate_pub = rospy.Publisher("/mavros_ned/estimate", Odometry, queue_size=1)


    def send_ned_estimate(self, event):

        # Fill out the message and publish.
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

        # Get the position data and populate the vector.
        self.position_vec_enu[0][0] = msg.pose.position.x
        self.position_vec_enu[1][0] = msg.pose.position.y
        self.position_vec_enu[2][0] = msg.pose.position.z

        # Rotate the vector from ENU to NED and store in class variable for use later.
        self.position_vec_ned = self.enu_to_ned(self.position_vec_enu)
        
        # Get the quaternion orientation from the message.
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)

        # Convert the quaternion to euler angles. 
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # Populate the FLU vector.
        self.euler_vec_flu[0][0] = euler[0]
        self.euler_vec_flu[1][0] = euler[1]
        self.euler_vec_flu[2][0] = euler[2]

        # Rotate from FLU to FRD and store in class variable for use later.
        euler_vec_frd = self.flu_to_frd(self.euler_vec_flu)

        # Convert back to quaternion and store in class variable for use later.
        self.quaternion_frd = tf.transformations.quaternion_from_euler(euler_vec_frd[0][0], euler_vec_frd[1][0], euler_vec_frd[2][0])
        # now = rospy.get_time()
        # delay = now - then
        # print(delay)


    def mavros_velocity_callback(self, msg):

        # Populate the RFU linear velocity vector.
        self.velocity_vec_lin_rfu[0][0] = msg.twist.linear.x
        self.velocity_vec_lin_rfu[1][0] = msg.twist.linear.y
        self.velocity_vec_lin_rfu[2][0] = msg.twist.linear.z

        # Rotate from RFU to FRD and store in class variable for use later.
        self.velocity_vec_lin_frd = self.enu_to_ned(self.velocity_vec_lin_rfu)

        # Populate FLU angular velocity vector.
        self.velocity_vec_ang_flu[0][0] = msg.twist.angular.x
        self.velocity_vec_ang_flu[1][0] = msg.twist.angular.y
        self.velocity_vec_ang_flu[2][0] = msg.twist.angular.z

        # Rotate from FLU to FRD and store in class variable for use later.
        self.velocity_vec_ang_frd = self.flu_to_frd(self.velocity_vec_ang_flu)


    def enu_to_ned(self, enu_vec):

        ned_vec = np.dot(self.R_enu_ned, enu_vec)
        return ned_vec


    def flu_to_frd(self, flu_vec):

        frd_vec = np.dot(self.R_flu_frd, flu_vec)
        return frd_vec


def main():
    # Initialize a node.
    rospy.init_node('mavros_ned')

    # Create instance of MavrosNED class.
    mavros_ned = MavrosNED()

    # Spin.
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
