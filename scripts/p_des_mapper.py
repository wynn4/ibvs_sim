#! /usr/bin/env python

## Simple ROS node that publishes desired pixel locations in the virtual level frame
## JSW April 2018

import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from aruco_localization.msg import FloatList
from geometry_msgs.msg import Point
import numpy as np
import cv2
import tf
import time


class FeatureMapper(object):

    def __init__(self):

        # Desired feature locations in the actual camera frame
        p_des_final_outer = rospy.get_param('~p_des_outer', [0., 0., 0., 0., 0., 0., 0., 0.])
        p_des_final_inner = rospy.get_param('~p_des_inner', [0., 0., 0., 0., 0., 0., 0., 0.])

        ## initialize other class variables
        
        # matrices to hold corner data
        self.uv_bar_lf = np.zeros((4,2))  # pixel coords (u,v) of the corner points in the virtual level frame 

        self.f_row = np.zeros((1,4))

        # self.matrix = np.zeros((2,2), dtype=np.float32)

        # camera params
        self.K = np.zeros((3,3))
        self.d = np.zeros(5)
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.f = 0.0

        # desired pixel coords 
        # [u1, v1, u2, v2, u3, v3, u4, v4].T  8x1
        self.p_des_outer = np.array(p_des_final_outer, dtype=np.float32).reshape(4,2)
        self.p_des_inner = np.array(p_des_final_inner, dtype=np.float32).reshape(4,2)
        # self.p_des0 = np.zeros((4,2), dtype=np.float32)

        # copter average attitude roll and pitch
        self.phi_avg = 0.0
        self.theta_avg = 0.0

        # side length in pixels of the marker when IBVS first becomes active
        self.side_length0 = 0.0

        # mounting angle offsets
        phi_m = 0.0    # roll relative to the body frame
        theta_m = 0.0  # pitch '...'
        psi_m = 0.0    # yaw '...'

        ## define fixed rotations
        sphi_m = np.sin(phi_m)
        cphi_m = np.cos(phi_m)
        stheta_m = np.sin(theta_m)
        ctheta_m = np.cos(theta_m)
        spsi_m = np.sin(psi_m )
        cpsi_m = np.cos(psi_m )

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
        self.R_c_vlc = np.zeros((3,3))

        self.ibvs_active = False
        # self.p_des0_set = False
        self.p_des_expressed_in_vlf = False

        # initialize publishers
        self.uv_bar_des_pub_outer = rospy.Publisher('/ibvs/pdes_outer', FloatList, queue_size=1)
        self.uv_bar_des_msg_outer = FloatList()

        self.uv_bar_des_pub_inner = rospy.Publisher('/ibvs/pdes_inner', FloatList, queue_size=1)
        self.uv_bar_des_msg_inner = FloatList()

        # initialize subscribers
        # self.corner_pix_sub = rospy.Subscriber('/aruco/marker_corners', FloatList, self.corners_callback)
        # self.attitude_sub = rospy.Subscriber('/quadcopter/estimate', Odometry, self.attitude_callback)
        self.avg_attitude_sub = rospy.Subscriber('/quadcopter/attitude_avg', Point, self.avg_attitude_callback)
        self.marker_corners_sub = rospy.Subscriber('/aruco/marker_corners_outer', FloatList, self.corners_callback)
        self.camera_info_sub = rospy.Subscriber('/quadcopter/camera/camera_info', CameraInfo, self.camera_info_callback)
        self.ibvs_active_sub = rospy.Subscriber('/quadcopter/ibvs_active', Bool, self.ibvs_active_callback)

         # Initialize timers.
        self.p_des_rate = 3.0
        self.p_des_timer = rospy.Timer(rospy.Duration(1.0/self.p_des_rate), self.send_p_des)


    def send_p_des(self, event):

        # Fill out the messages.
        self.uv_bar_des_msg_outer.header.frame_id = 'level-frame_corners_desired_outer'
        self.uv_bar_des_msg_outer.header.stamp = rospy.Time.now()
        self.uv_bar_des_msg_outer.data = self.p_des_outer.flatten().tolist()  # flatten and convert to list type

        self.uv_bar_des_msg_inner.header.frame_id = 'level-frame_corners_desired_inner'
        self.uv_bar_des_msg_inner.header.stamp = rospy.Time.now()
        self.uv_bar_des_msg_inner.data = self.p_des_inner.flatten().tolist()

        # Publish.
        self.uv_bar_des_pub_outer.publish(self.uv_bar_des_msg_outer)
        self.uv_bar_des_pub_inner.publish(self.uv_bar_des_msg_inner)


    def avg_attitude_callback(self, msg):

        # update class variables
        self.phi_avg = msg.x
        self.theta_avg = msg.y
        self.psi_avg = msg.z

        # p_des_vlf = self.transform_to_vlf(self.p_des)

        # self.uv_bar_des_msg.header.frame_id = 'level-frame_corners_desired'
        # self.uv_bar_des_msg.header.stamp = rospy.Time.now()
        # self.uv_bar_des_msg.data = p_des_vlf.flatten().tolist()  # flatten and convert to list type

        # # publish
        # self.uv_bar_des_pub.publish(self.uv_bar_des_msg)


    def ibvs_active_callback(self, msg):

        self.ibvs_active = msg.data


    def corners_callback(self, msg):

        if self.ibvs_active:

            if not self.p_des_expressed_in_vlf:
                self.p_des_outer = self.transform_to_vlf(self.p_des_outer)
                self.p_des_inner = self.transform_to_vlf(self.p_des_inner)

                self.p_des_expressed_in_vlf = True
            # if not self.p_des0_set:

            #     # Fill up p_des0 with the level frame corners where they currently appear
            #     self.p_des0[0][0] = msg.data[8]
            #     self.p_des0[0][1] = msg.data[9]
            #     self.p_des0[1][0] = msg.data[10]
            #     self.p_des0[1][1] = msg.data[11]
            #     self.p_des0[2][0] = msg.data[12]
            #     self.p_des0[2][1] = msg.data[13]
            #     self.p_des0[3][0] = msg.data[14]
            #     self.p_des0[3][1] = msg.data[15]

            #     # get the average side length of the square formed by the marker corners
            #     s1 = np.linalg.norm(self.p_des0[0][:] - self.p_des0[1][:])
            #     s2 = np.linalg.norm(self.p_des0[1][:] - self.p_des0[2][:])
            #     s3 = np.linalg.norm(self.p_des0[2][:] - self.p_des0[3][:])
            #     s4 = np.linalg.norm(self.p_des0[3][:] - self.p_des0[0][:])

            #     self.side_length0 = (s1 + s2 + s3 + s4) / 4.0

            #     # get the center of the group of marker corners
            #     center = np.array([[np.sum(self.p_des0[:,0])/4.0], [np.sum(self.p_des0[:,1])/4.0]]).reshape(1,2)

            #     # use the center and side length to make a square of pixel locations that is square (aligned) with the camera frame
            #     self.p_des0 = self.make_a_square(center, self.side_length0)

            #     # set the flag
            #     self.p_des0_set = True


    def transform_to_vlf(self, corners):

        # corners is a 4x2 matirx of undistorted center-relative corner pixel locations
        # store in a 3x4 matrix augmented with focal length (for convienience) and save for later
        uvf = np.concatenate((corners.T, self.f_row), axis=0)    # 3x4

        # setup rotations

        # pre-evaluate sines and cosines for rotation matrix
        sphi = np.sin(self.phi_avg)
        cphi = np.cos(self.phi_avg)
        stheta = np.sin(self.theta_avg)
        ctheta = np.cos(self.theta_avg)

        R_v1_v2 = np.array([[ctheta, 0., -stheta],
                            [0., 1., 0.],
                            [stheta, 0., ctheta]])

        R_v2_b = np.array([[1., 0., 0.],
                           [0., cphi, sphi],
                           [0., -sphi, cphi]])

        R_v1_b = np.dot(R_v2_b, R_v1_v2)

        # compute the whole rotation from camera frame to the virtual level frame
        self.R_c_vlc = self.R_m_c.dot(self.R_b_m.dot(R_v1_b.dot(self.R_vlc_v1))).T    # R_c_vlc = R_vlc_c.T

        # pass pixel locations through the rotation same way it's done in eq(14)
        
        # first, get the terms of eq(14)

        # this is how you'd do it one pixel (u, v) at a time:  
        # hom = np.dot(self.R_c_vlc, np.array([[corners_undist[0][0]],[corners_undist[0][1]],[self.f]]))  # this is a homography?? 3x1
        # x_term = hom[0][0]
        # y_term = hom[1][0]
        # denom = hom[2][0]

        # u_bar = self.f * (x_term/denom)
        # v_bar = self.f * (y_term/denom)

        # we'll do it all four corners at the same time:
        hom = np.dot(self.R_c_vlc, uvf) # 3x4
        
        # populate the matrix of (u,v) pixel coordinates in the virtual-level-frame
        self.uv_bar_lf[0][0] = self.f * (hom[0][0]/hom[2][0])   # u1
        self.uv_bar_lf[0][1] = self.f * (hom[1][0]/hom[2][0])   # v1

        self.uv_bar_lf[1][0] = self.f * (hom[0][1]/hom[2][1])   # u2
        self.uv_bar_lf[1][1] = self.f * (hom[1][1]/hom[2][1])   # v2

        self.uv_bar_lf[2][0] = self.f * (hom[0][2]/hom[2][2])   # u3
        self.uv_bar_lf[2][1] = self.f * (hom[1][2]/hom[2][2])   # v3

        self.uv_bar_lf[3][0] = self.f * (hom[0][3]/hom[2][3])   # u4
        self.uv_bar_lf[3][1] = self.f * (hom[1][3]/hom[2][3])   # v4

        # return a copy of 
        return self.uv_bar_lf.copy()


    def make_a_square(self, center, side_length):

        diag = np.sqrt(2.0) * side_length

        p0 = center.T + np.array([[-np.sqrt(2)/2],[-np.sqrt(2)/2]]) * (diag / 2.0)
        p1 = center.T + np.array([[np.sqrt(2)/2],[-np.sqrt(2)/2]]) * (diag / 2.0)
        p2 = center.T + np.array([[np.sqrt(2)/2],[np.sqrt(2)/2]]) * (diag / 2.0)
        p3 = center.T + np.array([[-np.sqrt(2)/2],[np.sqrt(2)/2]]) * (diag / 2.0)

        square = np.array([p0.T, p1.T, p2.T, p3.T]).reshape(4,2)

        return square




    def camera_info_callback(self, msg):

        # get the Camera Matrix K and distortion params d
        self.K = np.array(msg.K, dtype=np.float32).reshape((3,3))
        self.d = np.array(msg.D, dtype=np.float32)

        self.fx = self.K[0][0]
        self.fy = self.K[1][1]
        self.cx = self.K[0][2]
        self.cy = self.K[1][2]

        self.f = (self.fx + self.fy) / 2.0

        self.f_row[0][:] = self.f

        # just get this data once
        self.camera_info_sub.unregister()
        print("p_des_mapper: Got camera info!")


def main():
    # initialize a node
    rospy.init_node('pdes_mapper')

    # create instance of LevelFrameMapper class
    mapper = FeatureMapper()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # OpenCV cleanup
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
