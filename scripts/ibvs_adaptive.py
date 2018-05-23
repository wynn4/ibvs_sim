#! /usr/bin/env python

## Simple ROS node that subcribes to pixel data in the virtual level frame
## and computes image-based visual servoing control
##
## Sources:
## Lee et al. "Autonomous Landing of a VTOL UAV on a Moving Platform Using Image-based Visual Servoing"
## Corke, Peter "Robotics, Vision and Control"
##
## JSW Dec 2017

import rospy
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from aruco_localization.msg import FloatList
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import tf
import time


class ImageBasedVisualServoing(object):

    def __init__(self):

        # load ROS params
        # p_des = rospy.get_param('~p_des', [0., 0., 0., 0., 0., 0., 0., 0.])

        ## initialize other class variables

        # if set to True, IBVS is computed according to eq. 15.11 from Corke
        # if set to False, IBVS is computed according to eq. 10 from Lee
        self.inverse_method = True

        # if set True, IBVS is computed using the whole 2x6 image Jacobian (standard)
        # if set False, IBVS is computed using the reduced 2x4 4-dof image Jacobian (way better in this case!)
        self.six_dof = False

        # IBVS saturation values (very conservative)
        self.u_max = 10.0
        self.v_max = 10.0
        self.w_max = 7.0
        self.psidot_max = np.radians(22.5)

        # image size
        # Initialize to something non-zero
        self.img_w = 640
        self.img_h = 480

        # camera params
        self.K = np.zeros((3,3))
        self.d = np.zeros(5)
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.f = 1000.0  # initialize to ge greater than zero

        pixel_um = 3.75  # pixel size in micrometers
        pixel_size = pixel_um * 1e-6  # pixel size in meters

        # copter attitude roll and pitch
        # self.phi = 0.0
        # self.theta = 0.0
        self.altitude = 0.0

        # distance (height) to the ArUco
        self.z_c = 10.0  # initialize to be greater than zero

        # lambda from Corke eq. 15.11 (turning param)
        self.lam2DOF = np.array([0.5, 0.5], dtype=np.float32).reshape(2,1)  # 2x1
        self.lam4DOF = np.array([0.5, 0.5, 0.7, 0.5], dtype=np.float32).reshape(4,1)  # 4x1

        self.p_centroid = np.array([0.0, 0.0], dtype=np.float32).reshape(2,1)
        self.p_des_centroid = np.array([0.0, 0.0], dtype=np.float32).reshape(2,1)
        self.radius_pix = 0.0
        self.centroid_radius_inner = 100.0
        self.centroid_radius_outer = 200.0
        self.mode_flag = 'IBVS_2DOF'


        # desired pixel coords 
        # [u1, v1, u2, v2, u3, v3, u4, v4].T  8x1
        self.p_des = np.zeros((8,1), dtype=np.float32)  # 8x1

        # rotation from the virtual level frame to the vehicle 1 frame
        self.R_vlc_v1 = np.array([[0., -1., 0.],
                                  [1., 0., 0.],
                                  [0., 0., 1.]])

        # rotation from the vehicle 1 frame to the body frame (just initialize it--will update later)
        # self.R_v1_b = np.eye(3)

        # initialize rotation from virtual level frame to body frame
        self.R_vlc_b = np.zeros((3,3))

        # initialize velocity command
        self.vel_cmd_msg = Twist()

        # initialize subscribers
        self.uv_bar_des_sub = rospy.Subscriber('/ibvs/pdes', FloatList, self.level_frame_desired_corners_callback)
        self.uv_bar_sub = rospy.Subscriber('/aruco/marker_corners', FloatList, self.level_frame_corners_callback)
        self.aruco_sub = rospy.Subscriber('/aruco/distance', Float32, self.aruco_distance_callback)
        self.state_sub = rospy.Subscriber('/quadcopter/estimate', Odometry, self.altitude_callback)
        self.camera_info_sub = rospy.Subscriber('/quadcopter/camera/camera_info', CameraInfo, self.camera_info_callback)

        # initialize publishers
        self.vel_cmd_pub = rospy.Publisher('/ibvs/vel_cmd', Twist, queue_size=1)


    def level_frame_corners_callback(self, msg):

        # Level-frame corners are elements 8 to 15 on this topic
        u1 = msg.data[8]
        v1 = msg.data[9]

        u2 = msg.data[10]
        v2 = msg.data[11]

        u3 = msg.data[12]
        v3 = msg.data[13]

        u4 = msg.data[14]
        v4 = msg.data[15]

        # compute the centroid of the corners
        u_centroid = (u1 + u2 + u3 + u4) / 4.0
        v_centroid = (v1 + v2 + v3 + v4) / 4.0

        # Compute distance between the centroid, and the centroid of p_des and get appropriate IBVS flag
        self.mode_flag = self.set_ibvs_mode(u_centroid, v_centroid)
        print self.mode_flag

        # t = time.time()

        # compute z_c according to eq(15)
        # z_c = self.compute_dist(msg.data)

        if self.mode_flag == 'IBVS_2DOF':
            # formulate 2 DOF image Jacobians according to eq(5) but ignore all but the vx and vy terms
            Jp1 = np.array([[-self.f/self.z_c, 0.0],
                            [0.0, -self.f/self.z_c]])  # 2x2

            Jp2 = np.array([[-self.f/self.z_c, 0.0],
                            [0.0, -self.f/self.z_c]])  # 2x2

            Jp3 = np.array([[-self.f/self.z_c, 0.0],
                            [0.0, -self.f/self.z_c]])  # 2x2

            Jp4 = np.array([[-self.f/self.z_c, 0.0],
                            [0.0, -self.f/self.z_c]])  # 2x2

        else:
            # formulate image Jacobians according to eq(5) but ignore the angular x and y velocity terms
            Jp1 = np.array([[-self.f/self.z_c, 0.0, u1/self.z_c, v1],
                            [0.0, -self.f/self.z_c, v1/self.z_c, -u1]])  # 2x4
        
            Jp2 = np.array([[-self.f/self.z_c, 0.0, u2/self.z_c, v2],
                            [0.0, -self.f/self.z_c, v2/self.z_c, -u2]])  # 2x4

            Jp3 = np.array([[-self.f/self.z_c, 0.0, u3/self.z_c, v3],
                            [0.0, -self.f/self.z_c, v3/self.z_c, -u3]])  # 2x4

            Jp4 = np.array([[-self.f/self.z_c, 0.0, u4/self.z_c, v4],
                            [0.0, -self.f/self.z_c, v4/self.z_c, -u4]])  # 2x4

        # stack the Jacobians
        Jp = np.vstack((Jp1, Jp2, Jp3, Jp4))  # 8x4

        # formulate the error term e = p - p_des
        p = np.array([u1, v1, u2, v2, u3, v3, u4, v4]).reshape(8,1)
        # e = p - self.p_des  # 8x1

        # formulate the desired camera velocity vector rdot_des
        if self.mode_flag == 'IBVS_2DOF':
            e = self.p_des - p
            # v = lambda * pinv(Jp) * e
            rdot_des = self.lam2DOF * np.linalg.pinv(Jp).dot(e)  # 2x1
            # NOTE: In the future, I may want to try Corke's 2nd Order Jacobian (eq. 15.12)
        else:
            e = self.p_des - p
            # v = lambda * pinv(Jp) * e
            rdot_des = self.lam4DOF * np.linalg.pinv(Jp).dot(e)  # 4x1

        # e_avg = np.mean(e)
        # # print "Average pixel error: %f" % e_avg
        # print "error: "
        # print(e)
        # print "\n"
        # print "Jp inverse: "
        # print(np.linalg.pinv(Jp))
        # print "\n"
        
        # saturate and fill out the velocity command message
        if self.mode_flag == 'IBVS_2DOF':
            # Augment with a '0' to make a 3x1 (so we can rotate it).
            v_des_linear = np.array([[rdot_des[0][0]],
                                     [rdot_des[1][0]],
                                     [0.0]])

            # Rotate into the vehicle-1 frame
            v_des_v1_linear = np.dot(self.R_vlc_v1, v_des_linear)  # 3x1

            # Fill out the message
            self.vel_cmd_msg.linear.x = self.saturate(v_des_v1_linear[0][0], self.u_max, -self.u_max)
            self.vel_cmd_msg.linear.y = self.saturate(v_des_v1_linear[1][0], self.v_max, -self.v_max)
            self.vel_cmd_msg.linear.z = 0.0  # this term is always zero anyway in 2DOF mode

            # No angular velocities in this case
            self.vel_cmd_msg.angular.x = 0.0
            self.vel_cmd_msg.angular.y = 0.0
            self.vel_cmd_msg.angular.z = 0.0

        else:
            v_des_linear = np.array([[rdot_des[0][0]],
                                     [rdot_des[1][0]],
                                     [rdot_des[2][0]]])

            # Rotate into the vehicle-1 frame
            v_des_v1_linear = np.dot(self.R_vlc_v1, v_des_linear)  # 3x1

            # Fill out the message
            self.vel_cmd_msg.linear.x = self.saturate(v_des_v1_linear[0][0], self.u_max, -self.u_max)
            self.vel_cmd_msg.linear.y = self.saturate(v_des_v1_linear[1][0], self.v_max, -self.v_max)
            self.vel_cmd_msg.linear.z = self.saturate(v_des_v1_linear[2][0], self.w_max, -self.w_max)

            self.vel_cmd_msg.angular.x = 0.0
            self.vel_cmd_msg.angular.y = 0.0
            self.vel_cmd_msg.angular.z = self.saturate(rdot_des[3][0], self.psidot_max, -self.psidot_max)

        # publish
        self.vel_cmd_pub.publish(self.vel_cmd_msg)


    def level_frame_desired_corners_callback(self, msg):

        self.p_des[0][0] = msg.data[0]
        self.p_des[1][0] = msg.data[1]

        self.p_des[2][0] = msg.data[2]
        self.p_des[3][0] = msg.data[3]

        self.p_des[4][0] = msg.data[4]
        self.p_des[5][0] = msg.data[5]

        self.p_des[6][0] = msg.data[6]
        self.p_des[7][0] = msg.data[7]

        self.p_des_centroid[0][0] = (msg.data[0] + msg.data[2] + msg.data[4] + msg.data[6]) / 4.0
        self.p_des_centroid[1][0] = (msg.data[1] + msg.data[3] + msg.data[5] + msg.data[7]) / 4.0


    def set_ibvs_mode(self, u_centroid, v_centroid):

        self.p_centroid[0][0] = u_centroid
        self.p_centroid[1][0] = v_centroid

        # compute the pixel distance from the center of the corners to the center of the desired corner locations
        self.radius_pix = np.linalg.norm(self.p_centroid - self.p_des_centroid)

        # decide which mode we should be in based on how far away we are from the centroid of p_des
        if self.mode_flag == 'IBVS_2DOF':

            if self.radius_pix <= self.centroid_radius_inner:

                self.mode_flag = 'IBVS_4DOF'

                return self.mode_flag
            else:
                # Don't change the mode
                return self.mode_flag

        else:

            if self.radius_pix >= self.centroid_radius_outer:

                self.mode_flag = 'IBVS_2DOF'

                return self.mode_flag
            else:
                # Don't change the mode
                return self.mode_flag


    def compute_dist(self, corners):

        # This function included for completeness but in reality, I'm computing this same thing in 
        # ArucoLocalizer.cpp and using that instead.

        # get the distance in pixels between each of the corners
        Ls_1_2 = np.linalg.norm(np.array([[corners[0]],[corners[1]]]) - np.array([[corners[2]],[corners[3]]]))
        Ls_2_3 = np.linalg.norm(np.array([[corners[2]],[corners[3]]]) - np.array([[corners[4]],[corners[5]]]))
        Ls_3_4 = np.linalg.norm(np.array([[corners[4]],[corners[5]]]) - np.array([[corners[6]],[corners[7]]]))
        Ls_4_1 = np.linalg.norm(np.array([[corners[6]],[corners[7]]]) - np.array([[corners[0]],[corners[1]]]))

        # take the average
        Ls = (Ls_1_2 + Ls_2_3 + Ls_3_4 + Ls_4_1) / 4.0

        # compute distance to the target z_c (eq 15)
        z_c = (self.Lc * self.f) / Ls

        return z_c

        
    def altitude_callback(self, msg):

        self.altitude = -msg.pose.pose.position.z


    def camera_info_callback(self, msg):

        # get the Camera Matrix K and distortion params d
        self.K = np.array(msg.K, dtype=np.float32).reshape((3,3))
        self.d = np.array(msg.D, dtype=np.float32)

        self.fx = self.K[0][0]
        self.fy = self.K[1][1]
        self.cx = self.K[0][2]
        self.cy = self.K[1][2]

        self.f = (self.fx + self.fy) / 2.0

        self.img_w = msg.width
        self.img_h = msg.height

        # just get this data once
        self.camera_info_sub.unregister()
        print("IBVS: Got camera info!")


    def aruco_distance_callback(self, msg):

        # check for NaNs coming from ArUco distance
        if np.isnan(msg.data):
            # print "Nan!"
            z_c = self.altitude

            if z_c >= 3.0 and self.inverse_method == False:
                self.z_c = 3.0
            else:
               self.z_c = z_c

            return

        # all we need is distance to the ArUco, z_c
        z_c = msg.data

        if z_c >= 3.0 and self.inverse_method == False:
            self.z_c = 3.0
        else:
            self.z_c = z_c


    def saturate(self, x, maximum, minimum):
        if(x > maximum):
            rVal = maximum
        elif(x < minimum):
            rVal = minimum
        else:
            rVal = x

        return rVal


def main():
    # initialize a node
    rospy.init_node('ibvs')

    # create instance of ImageBasedVisualServoing class
    ibvs = ImageBasedVisualServoing()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # OpenCV cleanup
    # if ibvs.show:
    #     cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
