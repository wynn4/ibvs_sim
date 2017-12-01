#! /usr/bin/env python

## Simple ROS node that subcribes to pixel data in the fixed camera frame and
## transforms it to be expressed in virtual level camera frame
## JSW Nov 2017

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
from aruco_localization.msg import FloatList
import dynamic_reconfigure.client
import cv2
import math
import numpy as np


class LevelFrameMapper(object):

    def __init__(self):

        # load ROS params

        ## initialize other class variables

        # corners
        self.corner_0 = np.zeros(2)
        self.corner_1 = np.zeros(2)
        self.corner_2 = np.zeros(2)
        self.corner_3 = np.zeros(2)

        # copter attitude roll and pitch
        self.phi = 0.0
        self.theta = 0.0 

        # mounting angle offsets
        self.phi_m = 0.0    # roll relative to the body frame
        self.theta_m = 0.0  # pitch '...'
        self.psi_m = 0.0    # yaw '...'

        ## define fixed rotations

        sphi_m = np.sin(self.phi_m)
        cphi_m = np.cos(self.phi_m)
        stheta_m = np.sin(self.theta_m)
        ctheta_m = np.cos(self.theta_m)
        spsi_m = np.sin(self.psi_m)
        cpsi_m = np.cos(self.psi_m)

        # rotation from the virtual level frame to the vehicle 1 frame
        self.R_vlc_v1 = np.array([[0., -1., 0.],
                                  [1., 0., 0.],
                                  [0., 0., 1.]])

        # rotation from the body frame to the camera mount frame (same form as rotation from the vehicle to the body frame)
        self.R_b_m = np.array([[ctheta_m*cpsi_m, ctheta_m*spsi_m, -stheta_m],
                               [sphi_m*stheta_m*cpsi_m-cphi_m*spsi_m, sphi_m*stheta_m*spsi_m+cphi_m*cpsi_m, sphi_m*ctheta_m],
                               [cphi_m*stheta_m*cpsi_m+sphi_m*spsi_m, cphi_m*stheta_m*spsi_m-sphi_m*cpsi_m, cphi_m*ctheta_m]])

        # rotation from the mount frame to the camera frame
        self.R_m_c = np.array([[0., 1., 0.],
                               [-1., 0., 0.],
                               [0., 0., 1.]])


        # rotation from the vehicle 1 frame to the body frame (just initialize it--will update later)
        # self.R_v1_b = np.eye(3)

        # initialize rotation from camera frame to virtual level frame
        self.R_c_vlc = np.zeros(3)


        # initialize CVBridge object

        # initialize subscriber
        self.corner_pix_sub = rospy.Subscriber('/aruco/marker_corners', FloatList, self.corners_callback)


        # initialize publishers

        # initialize timer
        # self.exposure_update_rate = 1.0
        # self.update_timer = rospy.Timer(rospy.Duration(1.0/self.exposure_update_rate), self.process_image)


    def corners_callback(self, msg):

        # update corner pixel locations
        self.corner_0[0] = msg.data[0]  # x pixel
        self.corner_0[1] = msg.data[1]  # y pixel

        self.corner_1[0] = msg.data[2]  # x pixel
        self.corner_1[1] = msg.data[3]  # y pixel

        self.corner_2[0] = msg.data[4]  # x pixel
        self.corner_2[1] = msg.data[5]  # y pixel

        self.corner_3[0] = msg.data[6]  # x pixel
        self.corner_3[1] = msg.data[7]  # y pixel

        # TODO undistort the corner locations??

        # rotate pixel locations into the level frame




    def attitude_callback(self, msg):

        # get the copter's roll/pitch

        # pre-evaluate sines and cosines for rotation matrix
        sphi = np.sin(self.phi)
        cphi = np.cos(self.phi)
        stheta = np.sin(self.theta)
        ctheta = np.cos(self.theta)

        R_v1_v2 = np.array([[ctheta, 0., -stheta],
                            [0., 1., 0.],
                            [stheta, 0., ctheta]])

        R_v2_b = np.array([[1., 0., 0.],
                           [0., cphi, sphi],
                           [0., -sphi, cphi]])

        R_v1_b = np.dot(R_v2_b, R_v1_v2)

        # compute the whole rotation from camera frame to the virtual level frame
        # R_c_vlc = R_vlc_c.T
        self.R_c_vlc = self.R_m_c.dot(self.R_b_m.dot(R_v1_b.dot(self.R_vlc_v1))).T




def main():
    # initialize a node
    rospy.init_node('level_frame_pixel_publisher')

    # create instance of LevelFrameMapper class
    mapper = LevelFrameMapper()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # OpenCV cleanup
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
