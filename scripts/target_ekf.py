#! /usr/bin/env python

## Target Velocity EKF
## JSW June 2018

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
import time


class TargetEKF(object):

    def __init__(self):

        ## Load ROS params
        # Camera Offsets
        self.delta_x = rospy.get_param('~delta_x', 0.0)
        self.delta_y = rospy.get_param('~delta_y', 0.0)
        self.delta_z = rospy.get_param('~delta_z', 0.0)


        ## Static Transformations
        # Transform from camera to mount frame
        self.T_c_m = np.array([[0, -1, 0, 0],
                               [1, 0, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]], dtype=np.float32)

        # Transform from mount to body frame (assumes camera mount is perfectly aligned with body frame)
        self.T_m_b = np.array([[1, 0, 0, self.delta_x],
                               [0, 1, 0, self.delta_y],
                               [0, 0, 1, self.delta_z],
                               [0, 0, 0, 1]], dtype=np.float32)

        
        ## Other Transformations
        # Transform from body to vehicle-1 frame
        self.T_b_v = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]], dtype=np.float32)

        # Transform from vehicle-1 to inertial frame
        # Transform from body to vehicle-1 frame
        self.T_v_i = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]], dtype=np.float32)

        self.R_b_v = np.eye(3, dtype=np.float32)
        self.R_v_b = np.eye(3, dtype=np.float32)

        # Initialize euler angles
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        # Initialize position
        self.Pn = 0.0
        self.Pe = 0.0
        self.Pd = 0.0

        # 4x1 Vectors to hold our measurements
        self.Z_c = np.array([0, 0, 0, 1], dtype=np.float32).reshape(4,1)
        self.Z_i = np.array([0, 0, 0, 1], dtype=np.float32).reshape(4,1)

        
        ## EKF Data

        # State vector.
        self.x_hat = np.zeros((4,1), dtype=np.float32)

        # Propagation Jacobian
        self.F = np.eye(4, dtype=np.float32)

        # Covariance Matrix
        self.P = np.diag([1.e10, 1.e10, 1.e10, 1.e10])

        # Model Uncertainty
        self.Q = np.diag([1.e1, 1.e1, 1.e1])

        # Gamma(T)
        self.Gamma = np.zeros((4,4), dtype=np.float32)
        
        # Kalman gain.
        self.K = np.zeros((4,2), dtype=np.float32)

        # Measurement Jacobian
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=np.float32)


        # Subscribe to the ArUco's pose in the camera frame
        self.target_sub = rospy.Subscriber('/aruco/estimate', PoseStamped, self.target_callback)

        self.euler_sub = rospy.Subscriber('/quadcopter/euler', Vector3Stamped, self.euler_callback)
        self.position_sub = rospy.Subscriber('/quadcopter/ground_truth/odometry/NED', Odometry, self.position_callback)

        self.estimate_rate = 50.0
        self.estimate_timer = rospy.Timer(rospy.Duration(1.0/self.estimate_rate), self.send_estimate)


    def send_estimate(self, event):

        # Predict Step

    
    def target_callback(self, msg):

        # Grab the ArUco's position in the camera frame
        self.Z_c[0][0] = msg.pose.position.x
        self.Z_c[1][0] = msg.pose.position.y
        self.Z_c[2][0] = msg.pose.position.z

        # Transform the ArUco's position to be expressed in the inertial frame
        self.transform_c_to_i()

        # print "Target North: %f" % self.Z_i[0][0]
        # print "Target East: %f" % self.Z_i[1][0]
        # print "Target Down: %f" % self.Z_i[2][0]
        # print "\n"

        # Run a measurement update step on our EKF
        self.update_step()


    def transform_c_to_i(self):

        # Update transformations.
        self.T_v_i[0][3] = self.Pn
        self.T_v_i[1][3] = self.Pe
        self.T_v_i[2][3] = self.Pd

        # Pre-evaluate sines and cosines.
        sphi = np.sin(self.phi)
        cphi = np.cos(self.phi)
        stheta = np.sin(self.theta)
        ctheta = np.cos(self.theta)
        spsi = np.sin(self.psi)
        cpsi = np.cos(self.psi)

        # Update rotation from vehicle to body frame.
        self.R_v_b[0][0] = ctheta * cpsi
        self.R_v_b[0][1] = ctheta * spsi
        self.R_v_b[0][2] = -stheta
        self.R_v_b[1][0] = sphi * stheta *cpsi - cphi * spsi
        self.R_v_b[1][1] = sphi * stheta * spsi + cphi*cpsi
        self.R_v_b[1][2] = sphi * ctheta
        self.R_v_b[2][0] = cphi * stheta * cpsi + sphi * spsi
        self.R_v_b[2][1] = cphi * stheta * spsi - sphi * cpsi
        self.R_v_b[2][2] = cphi * ctheta

        self.R_b_v = self.R_v_b.T
        self.T_b_v[0:3,0:3] = self.R_b_v

        # Compute the whole transformation from camera frame to inertial frame.
        self.T_c_i = np.dot(self.T_v_i, np.dot(self.T_b_v, np.dot(self.T_m_b, self.T_c_m)))

        # Transform the measurement expressed in the camera frame to be expressed in the inertial frame.
        self.Z_i = np.dot(self.T_c_i, self.Z_c)


    def update_step(self):

        # Compute the Kalman Gain.
        self.K = np.dot(self.P, np.dot(self.H.T, np.linalg.inv(np.dot(self.H, np.dot(self.P, self.H.T)) + self.R)))

        # Update the estimate.
        self.x_hat = self.x_hat + np.dot(self.K, (self.Z_i[0:2] - self.x_hat[0:2]))

        # Update covariance.
        self.P = np.dot((self.I - np.dot(self.K, self.H)), self.P)


    def euler_callback(self, msg):

        # Pull off the euler angles.
        self.phi = msg.vector.x
        self.theta = msg.vector.y
        self.psi = msg.vector.z


    def position_callback(self, msg):

        # Pull off the NED position data
        self.Pn = msg.pose.pose.position.x
        self.Pe = msg.pose.pose.position.y
        self.Pd = msg.pose.pose.position.z






        


   


def main():
    # initialize a node
    rospy.init_node('target_ekf')

    # create instance of TargetEKF class
    ekf = TargetEKF()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
