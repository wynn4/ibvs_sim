#! /usr/bin/env python

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import String
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from aruco_localization.msg import FloatList
import numpy as np
import scipy.io
import cv2
import tf
import time
from os import path


class SaveMatData(object):

    def __init__(self):

        # File string
        file_str_outer = rospy.get_param('~filename_outer', 'ibvs_data_outer.mat')
        file_str_inner = rospy.get_param('~filename_inner', 'ibvs_data_inner.mat')
        self.outfile_str_outer = path.expanduser('~/') + file_str_outer
        self.outfile_str_inner = path.expanduser('~/') + file_str_inner


        # Giant Arrays For storing Pixel Data
        self.data_array_outer = np.zeros((50000,32))
        self.data_array_inner = np.zeros((50000,32))

        self.ibvs_active = False
        self.line_count = 0
        self.data_saved = False

        # flag
        self.status_flag = ''
        self.mode = 0.0

        self.line_count_outer = 0
        self.line_count_inner = 0

        self.phi = 0.0
        self.theta = 0.0


        # desired pixel coords 
        # [u1, v1, u2, v2, u3, v3, u4, v4].T  8x1
        self.p_des_outer = np.array([-200, -200, 200, -200, 200, 200, -200, 200], dtype=np.float32).reshape(8,1)
        self.p_des_inner = np.array([-200, -200, 200, -200, 200, 200, -200, 200], dtype=np.float32).reshape(8,1)

        # initialize subscribers
        self.uv_bar_outer_sub = rospy.Subscriber('/aruco/marker_corners_outer', FloatList, self.corners_outer_callback)
        self.uv_bar_inner_sub = rospy.Subscriber('/aruco/marker_corners_inner', FloatList, self.corners_inner_callback)
        self.uv_bar_des_outer_sub = rospy.Subscriber('/ibvs/pdes_outer', FloatList, self.level_frame_desired_corners_outer_callback)
        self.uv_bar_des_inner_sub = rospy.Subscriber('/ibvs/pdes_inner', FloatList, self.level_frame_desired_corners_inner_callback)
        self.attitude_sub = rospy.Subscriber('/quadcopter/estimate', Odometry, self.attitude_callback)
        self.ibvs_active_sub = rospy.Subscriber('/quadcopter/ibvs_active', Bool, self.ibvs_active_callback)
        self.state_machine_status_sub = rospy.Subscriber('/status_flag', String, self.status_flag_callback)
        self.save_now_sub = rospy.Subscriber('/save_ibvs_mat_now', Bool, self.save_data_now_callback)


    def corners_outer_callback(self, msg):

        # desired pixel locations
        p1_des = np.array([[self.p_des_outer[0][0]],[self.p_des_outer[1][0]]])
        # u1_des = self.p_des_outer[0][0]
        # v1_des = self.p_des_outer[1][0]

        p2_des = np.array([[self.p_des_outer[2][0]],[self.p_des_outer[3][0]]])
        # u2_des = self.p_des_outer[2][0]
        # v2_des = self.p_des_outer[3][0]

        p3_des = np.array([[self.p_des_outer[4][0]],[self.p_des_outer[5][0]]])
        # u3_des = self.p_des_outer[4][0]
        # v3_des = self.p_des_outer[5][0]

        p4_des = np.array([[self.p_des_outer[6][0]],[self.p_des_outer[7][0]]])
        # u4_des = self.p_des_outer[6][0]
        # v4_des = self.p_des_outer[7][0]

        # current pixel locations
        p1 = np.array([[msg.data[8]],[msg.data[9]]])
        # u1= msg.data[0]   # u1
        # v1= msg.data[1]   # v1

        p2 = np.array([[msg.data[10]],[msg.data[11]]])
        # u2= msg.data[2]   # u2
        # v2= msg.data[3]   # v2

        p3 = np.array([[msg.data[12]],[msg.data[13]]])
        # u3= msg.data[4]   # u3
        # v3= msg.data[5]   # v3

        p4 = np.array([[msg.data[14]],[msg.data[15]]])
        # u4= msg.data[6]   # u4
        # v4= msg.data[7]   # v4

        # Calculate the error(s) as the L2 norm
        e1 = np.linalg.norm(p1_des - p1)
        e2 = np.linalg.norm(p2_des - p2)
        e3 = np.linalg.norm(p3_des - p3)
        e4 = np.linalg.norm(p4_des - p4)

        time = rospy.get_time()
        # print(time)

        if self.line_count_outer < 50000 and (not self.data_saved):
            # error between p_des and p
            self.data_array_outer[self.line_count_outer][0] = time
            self.data_array_outer[self.line_count_outer][1] = e1
            self.data_array_outer[self.line_count_outer][2] = e2
            self.data_array_outer[self.line_count_outer][3] = e3
            self.data_array_outer[self.line_count_outer][4] = e4

            # raw corner data
            self.data_array_outer[self.line_count_outer][5] = msg.data[0]
            self.data_array_outer[self.line_count_outer][6] = msg.data[1]
            self.data_array_outer[self.line_count_outer][7] = msg.data[2]
            self.data_array_outer[self.line_count_outer][8] = msg.data[3]
            self.data_array_outer[self.line_count_outer][9] = msg.data[4]
            self.data_array_outer[self.line_count_outer][10] = msg.data[5]
            self.data_array_outer[self.line_count_outer][11] = msg.data[6]
            self.data_array_outer[self.line_count_outer][12] = msg.data[7]

            # level_corner data
            self.data_array_outer[self.line_count_outer][13] = msg.data[8]
            self.data_array_outer[self.line_count_outer][14] = msg.data[9]
            self.data_array_outer[self.line_count_outer][15] = msg.data[10]
            self.data_array_outer[self.line_count_outer][16] = msg.data[11]
            self.data_array_outer[self.line_count_outer][17] = msg.data[12]
            self.data_array_outer[self.line_count_outer][18] = msg.data[13]
            self.data_array_outer[self.line_count_outer][19] = msg.data[14]
            self.data_array_outer[self.line_count_outer][20] = msg.data[15]

            # p_des data
            self.data_array_outer[self.line_count_outer][21] = self.p_des_outer[0][0]
            self.data_array_outer[self.line_count_outer][22] = self.p_des_outer[1][0]
            self.data_array_outer[self.line_count_outer][23] = self.p_des_outer[2][0]
            self.data_array_outer[self.line_count_outer][24] = self.p_des_outer[3][0]
            self.data_array_outer[self.line_count_outer][25] = self.p_des_outer[4][0]
            self.data_array_outer[self.line_count_outer][26] = self.p_des_outer[5][0]
            self.data_array_outer[self.line_count_outer][27] = self.p_des_outer[6][0]
            self.data_array_outer[self.line_count_outer][28] = self.p_des_outer[7][0]

            # roll and pitch angles
            self.data_array_outer[self.line_count_outer][29] = self.phi
            self.data_array_outer[self.line_count_outer][30] = self.theta

            # state_machine mode flag
            self.data_array_outer[self.line_count_outer][31] = self.mode


        # increment
        self.line_count_outer += 1


        # elapsed = time.time() - t
        # hz_approx = 1.0/elapsed
        # print(hz_approx)


    def corners_inner_callback(self, msg):

        # desired pixel locations
        p1_des = np.array([[self.p_des_inner[0][0]],[self.p_des_inner[1][0]]])
        # u1_des = self.p_des_outer[0][0]
        # v1_des = self.p_des_outer[1][0]

        p2_des = np.array([[self.p_des_inner[2][0]],[self.p_des_inner[3][0]]])
        # u2_des = self.p_des_outer[2][0]
        # v2_des = self.p_des_outer[3][0]

        p3_des = np.array([[self.p_des_inner[4][0]],[self.p_des_inner[5][0]]])
        # u3_des = self.p_des_outer[4][0]
        # v3_des = self.p_des_outer[5][0]

        p4_des = np.array([[self.p_des_inner[6][0]],[self.p_des_inner[7][0]]])
        # u4_des = self.p_des_outer[6][0]
        # v4_des = self.p_des_outer[7][0]

        # current pixel locations
        p1 = np.array([[msg.data[8]],[msg.data[9]]])
        # u1= msg.data[0]   # u1
        # v1= msg.data[1]   # v1

        p2 = np.array([[msg.data[10]],[msg.data[11]]])
        # u2= msg.data[2]   # u2
        # v2= msg.data[3]   # v2

        p3 = np.array([[msg.data[12]],[msg.data[13]]])
        # u3= msg.data[4]   # u3
        # v3= msg.data[5]   # v3

        p4 = np.array([[msg.data[14]],[msg.data[15]]])
        # u4= msg.data[6]   # u4
        # v4= msg.data[7]   # v4

        # Calculate the error(s) as the L2 norm
        e1 = np.linalg.norm(p1_des - p1)
        e2 = np.linalg.norm(p2_des - p2)
        e3 = np.linalg.norm(p3_des - p3)
        e4 = np.linalg.norm(p4_des - p4)

        time = rospy.get_time()
        # print(time)

        if self.line_count_inner < 50000 and (not self.data_saved):
            # error between p_des and p
            self.data_array_inner[self.line_count_inner][0] = time
            self.data_array_inner[self.line_count_inner][1] = e1
            self.data_array_inner[self.line_count_inner][2] = e2
            self.data_array_inner[self.line_count_inner][3] = e3
            self.data_array_inner[self.line_count_inner][4] = e4

            # raw corner data
            self.data_array_inner[self.line_count_inner][5] = msg.data[0]
            self.data_array_inner[self.line_count_inner][6] = msg.data[1]
            self.data_array_inner[self.line_count_inner][7] = msg.data[2]
            self.data_array_inner[self.line_count_inner][8] = msg.data[3]
            self.data_array_inner[self.line_count_inner][9] = msg.data[4]
            self.data_array_inner[self.line_count_inner][10] = msg.data[5]
            self.data_array_inner[self.line_count_inner][11] = msg.data[6]
            self.data_array_inner[self.line_count_inner][12] = msg.data[7]

            # level_corner data
            self.data_array_inner[self.line_count_inner][13] = msg.data[8]
            self.data_array_inner[self.line_count_inner][14] = msg.data[9]
            self.data_array_inner[self.line_count_inner][15] = msg.data[10]
            self.data_array_inner[self.line_count_inner][16] = msg.data[11]
            self.data_array_inner[self.line_count_inner][17] = msg.data[12]
            self.data_array_inner[self.line_count_inner][18] = msg.data[13]
            self.data_array_inner[self.line_count_inner][19] = msg.data[14]
            self.data_array_inner[self.line_count_inner][20] = msg.data[15]

            # p_des data
            self.data_array_inner[self.line_count_inner][21] = self.p_des_inner[0][0]
            self.data_array_inner[self.line_count_inner][22] = self.p_des_inner[1][0]
            self.data_array_inner[self.line_count_inner][23] = self.p_des_inner[2][0]
            self.data_array_inner[self.line_count_inner][24] = self.p_des_inner[3][0]
            self.data_array_inner[self.line_count_inner][25] = self.p_des_inner[4][0]
            self.data_array_inner[self.line_count_inner][26] = self.p_des_inner[5][0]
            self.data_array_inner[self.line_count_inner][27] = self.p_des_inner[6][0]
            self.data_array_inner[self.line_count_inner][28] = self.p_des_inner[7][0]

            # roll and pitch angles
            self.data_array_inner[self.line_count_inner][29] = self.phi
            self.data_array_inner[self.line_count_inner][30] = self.theta

            # state_machine mode flag
            self.data_array_inner[self.line_count_inner][31] = self.mode


        # increment
        self.line_count_inner += 1


        # elapsed = time.time() - t
        # hz_approx = 1.0/elapsed
        # print(hz_approx)

    def level_frame_desired_corners_outer_callback(self, msg):

        self.p_des_outer[0][0] = msg.data[0]
        self.p_des_outer[1][0] = msg.data[1]

        self.p_des_outer[2][0] = msg.data[2]
        self.p_des_outer[3][0] = msg.data[3]

        self.p_des_outer[4][0] = msg.data[4]
        self.p_des_outer[5][0] = msg.data[5]

        self.p_des_outer[6][0] = msg.data[6]
        self.p_des_outer[7][0] = msg.data[7]


    def level_frame_desired_corners_inner_callback(self, msg):

        self.p_des_inner[0][0] = msg.data[0]
        self.p_des_inner[1][0] = msg.data[1]

        self.p_des_inner[2][0] = msg.data[2]
        self.p_des_inner[3][0] = msg.data[3]

        self.p_des_inner[4][0] = msg.data[4]
        self.p_des_inner[5][0] = msg.data[5]

        self.p_des_inner[6][0] = msg.data[6]
        self.p_des_inner[7][0] = msg.data[7]


    def camera_info_callback(self, msg):

        # get the image dimensions
        self.img_w = msg.width
        self.img_h = msg.height

        # set the level_frame to match the camera frame
        shape = (self.img_h, self.img_w, 3)
        self.level_frame = np.zeros(shape, np.uint8)

        # just get this data once
        self.camera_info_sub.unregister()
        print("Level_frame_visualizer: Got camera info!")


    def ibvs_active_callback(self, msg):
        
        self.ibvs_active = msg.data


    def status_flag_callback(self, msg):

        self.status_flag = msg.data


    def attitude_callback(self, msg):

        # get the quaternion orientation from the message
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        # convert to euler angles 
        euler = tf.transformations.euler_from_quaternion(quaternion)

        # update class variables
        self.phi = euler[0]
        self.theta = euler[1]


    def save_data_now_callback(self, msg):

        # Trim
        self.data_array_outer = self.data_array_outer[0:(self.line_count_outer)]
        self.data_array_inner = self.data_array_inner[0:(self.line_count_inner)]

        scipy.io.savemat(self.outfile_str_outer, mdict={'arr': self.data_array_outer})
        scipy.io.savemat(self.outfile_str_inner, mdict={'arr': self.data_array_inner})

        self.data_saved = True

        print "ibvs_mat_data_saver: data saved."

        # just get this data once
        self.save_now_sub.unregister()






def main():
    # initialize a node
    rospy.init_node('ibvs_mat_data_saver')

    # create instance of SaveMatData class
    saver = SaveMatData()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # Save off the data file
    # if visualizer.save_data:
    #     scipy.io.savemat(visualizer.outfile_str, mdict={'arr': visualizer.data_array_outer})


if __name__ == '__main__':
    main()
