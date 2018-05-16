#! /usr/bin/env python

## Simple ROS node that subcribes to pixel data in the virtual level camera frame
## and displays a visualization of the pixel data
## JSW Jan 2018

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


class LevelFrameVisualizer(object):

    def __init__(self):

        # load ROS params

        ## initialize other class variables

        # image size
        # Initialize to something non-zero
        self.img_w = 640
        self.img_h = 480

        # visualization params
        self.show = rospy.get_param('~show', True)
        shape = (self.img_h, self.img_w, 3)
        self.level_frame = np.zeros(shape, np.uint8)

        # plotting params
        self.save_data = rospy.get_param('~save_data', False)
        file_str = rospy.get_param('~filename', 'Desktop/data.mat')
        self.outfile_str = path.expanduser('~/') + file_str
        self.error_data = np.zeros((1000,5))
        self.ibvs_active = False
        self.line_count = 0

        # flag
        self.status_flag = ''

        # desired pixel coords 
        # [u1, v1, u2, v2, u3, v3, u4, v4].T  8x1
        self.p_des = np.array([-200, -200, 200, -200, 200, 200, -200, 200], dtype=np.float32).reshape(8,1)

        # matrix to hold pixel data
        self.uv_bar_lf = np.zeros((4,2))  # pixel coords (u,v) of the corner points in the virtual level frame
        self.corners = np.zeros((4,2))

        # initialize subscribers
        self.uv_bar_sub = rospy.Subscriber('/aruco/marker_corners_outer', FloatList, self.corners_callback)
        self.uv_bar_des_sub = rospy.Subscriber('/ibvs/pdes_outer', FloatList, self.level_frame_desired_corners_callback)
        self.camera_info_sub = rospy.Subscriber('/quadcopter/camera/camera_info', CameraInfo, self.camera_info_callback)
        self.ibvs_active_sub = rospy.Subscriber('/quadcopter/ibvs_active', Bool, self.ibvs_active_callback)
        self.state_machine_status_sub = rospy.Subscriber('/status_flag', String, self.status_flag_callback)


    def corners_callback(self, msg):

        # t = time.time()
        # populate corners matrix
        self.corners[0][0] = msg.data[0]
        self.corners[0][1] = msg.data[1]

        self.corners[1][0] = msg.data[2]
        self.corners[1][1] = msg.data[3]

        self.corners[2][0] = msg.data[4]
        self.corners[2][1] = msg.data[5]

        self.corners[3][0] = msg.data[6]
        self.corners[3][1] = msg.data[7]

        
        # populate the matrix of (u,v) pixel coordinates in the virtual-level-frame
        self.uv_bar_lf[0][0] = msg.data[8]   # u1
        self.uv_bar_lf[0][1] = msg.data[9]   # v1

        self.uv_bar_lf[1][0] = msg.data[10]   # u2
        self.uv_bar_lf[1][1] = msg.data[11]   # v2

        self.uv_bar_lf[2][0] = msg.data[12]   # u3
        self.uv_bar_lf[2][1] = msg.data[13]   # v3

        self.uv_bar_lf[3][0] = msg.data[14]   # u4
        self.uv_bar_lf[3][1] = msg.data[15]   # v4

        # print "data:"
        # print "uv_bar_lf: "
        # print(self.uv_bar_lf)
        # print "\n"

        # print "uv_bar_cam: "
        # print(corners_undist)
        # print "\n"

        if self.save_data and self.ibvs_active:

            # desired pixel locations
            p1_des = np.array([[self.p_des[0][0]],[self.p_des[1][0]]])
            # u1_des = self.p_des[0][0]
            # v1_des = self.p_des[1][0]

            p2_des = np.array([[self.p_des[2][0]],[self.p_des[3][0]]])
            # u2_des = self.p_des[2][0]
            # v2_des = self.p_des[3][0]

            p3_des = np.array([[self.p_des[4][0]],[self.p_des[5][0]]])
            # u3_des = self.p_des[4][0]
            # v3_des = self.p_des[5][0]

            p4_des = np.array([[self.p_des[6][0]],[self.p_des[7][0]]])
            # u4_des = self.p_des[6][0]
            # v4_des = self.p_des[7][0]

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

            # add a new line to the data matrix
            self.error_data[self.line_count][0] = time
            self.error_data[self.line_count][1] = e1
            self.error_data[self.line_count][2] = e2
            self.error_data[self.line_count][3] = e3
            self.error_data[self.line_count][4] = e4

            # increment
            self.line_count += 1


        if self.show:

            # clear out the frame
            self.level_frame.fill(0)

            # draw the original pixel coordinates
            cv2.circle(self.level_frame, (int(self.corners[0][0]), int(self.corners[0][1])), 10, (0, 0, 255), 2)
            cv2.circle(self.level_frame, (int(self.corners[1][0]), int(self.corners[1][1])), 10, (0, 0, 255), 2)
            cv2.circle(self.level_frame, (int(self.corners[2][0]), int(self.corners[2][1])), 10, (0, 0, 255), 2)
            cv2.circle(self.level_frame, (int(self.corners[3][0]), int(self.corners[3][1])), 10, (0, 0, 255), 2)

            # draw the level-frame pixel coordinates and desired coordinates
            cv2.circle(self.level_frame, (int(self.uv_bar_lf[0][0]+self.img_w/2.0), int(self.uv_bar_lf[0][1]+self.img_h/2.0)), 10, (0, 255, 255), 2)
            cv2.circle(self.level_frame, (int(self.uv_bar_lf[1][0]+self.img_w/2.0), int(self.uv_bar_lf[1][1]+self.img_h/2.0)), 10, (0, 255, 255), 2)
            cv2.circle(self.level_frame, (int(self.uv_bar_lf[2][0]+self.img_w/2.0), int(self.uv_bar_lf[2][1]+self.img_h/2.0)), 10, (0, 255, 255), 2)
            cv2.circle(self.level_frame, (int(self.uv_bar_lf[3][0]+self.img_w/2.0), int(self.uv_bar_lf[3][1]+self.img_h/2.0)), 10, (0, 255, 255), 2)

            cv2.circle(self.level_frame, (int(self.p_des[0][0]+self.img_w/2.0), int(self.p_des[1][0]+self.img_h/2.0)), 10, (0, 255, 0), 2)
            cv2.circle(self.level_frame, (int(self.p_des[2][0]+self.img_w/2.0), int(self.p_des[3][0]+self.img_h/2.0)), 10, (0, 255, 0), 2)
            cv2.circle(self.level_frame, (int(self.p_des[4][0]+self.img_w/2.0), int(self.p_des[5][0]+self.img_h/2.0)), 10, (0, 255, 0), 2)
            cv2.circle(self.level_frame, (int(self.p_des[6][0]+self.img_w/2.0), int(self.p_des[7][0]+self.img_h/2.0)), 10, (0, 255, 0), 2)

            cv2.line(self.level_frame, (int(self.uv_bar_lf[0][0]+self.img_w/2.0), int(self.uv_bar_lf[0][1]+self.img_h/2.0)), (int(self.p_des[0][0]+self.img_w/2.0), int(self.p_des[1][0]+self.img_h/2.0)), (255, 0, 0))
            cv2.line(self.level_frame, (int(self.uv_bar_lf[1][0]+self.img_w/2.0), int(self.uv_bar_lf[1][1]+self.img_h/2.0)), (int(self.p_des[2][0]+self.img_w/2.0), int(self.p_des[3][0]+self.img_h/2.0)), (255, 0, 0))
            cv2.line(self.level_frame, (int(self.uv_bar_lf[2][0]+self.img_w/2.0), int(self.uv_bar_lf[2][1]+self.img_h/2.0)), (int(self.p_des[4][0]+self.img_w/2.0), int(self.p_des[5][0]+self.img_h/2.0)), (255, 0, 0))
            cv2.line(self.level_frame, (int(self.uv_bar_lf[3][0]+self.img_w/2.0), int(self.uv_bar_lf[3][1]+self.img_h/2.0)), (int(self.p_des[6][0]+self.img_w/2.0), int(self.p_des[7][0]+self.img_h/2.0)), (255, 0, 0))

            # cross heirs
            cv2.line(self.level_frame, (int(-5.0 + self.img_w/2.0), int(self.img_h/2.0)), (int(5.0 + self.img_w/2.0), int(self.img_h/2.0)), (255, 255, 255))
            cv2.line(self.level_frame, (int(self.img_w/2.0), int(-5.0 + self.img_h/2.0)), (int(self.img_w/2.0), int(5.0 + self.img_h/2.0)), (255, 255, 255))

            center = np.array([[np.sum(self.corners[:,0])/4.0], [np.sum(self.corners[:,1])/4.0]]).reshape(1,2)
            cv2.line(self.level_frame, (int(-5.0 + center[0][0]), int(center[0][1])), (int(5.0 + center[0][0]), int(center[0][1])), (0, 0, 255))
            cv2.line(self.level_frame, (int(center[0][0]), int(-5.0 + center[0][1])), (int(center[0][0]), int(5.0 + center[0][1])), (0, 0, 255))

            # display the status flag
            cv2.putText(self.level_frame, "State Machine Status: " + self.status_flag, (50, 25),cv2.FONT_HERSHEY_PLAIN,1.0,(255,255,255),1)

            # add some text labels
            cv2.putText(self.level_frame, "Camera Frame",(50,55),cv2.FONT_HERSHEY_PLAIN,1.0,(0,0,255),1)
            cv2.putText(self.level_frame, "Virtual Level Frame",(50,75),cv2.FONT_HERSHEY_PLAIN,1.0,(0,255,255),1)
            cv2.putText(self.level_frame, "Desired Pixel Coordinates (VLF)",(50,95),cv2.FONT_HERSHEY_PLAIN,1.0,(0,255,0),1)

            # display the image
            cv2.imshow("level_frame_image", self.level_frame)
            cv2.waitKey(1)

        # elapsed = time.time() - t
        # hz_approx = 1.0/elapsed
        # print(hz_approx)

    def level_frame_desired_corners_callback(self, msg):

        self.p_des[0][0] = msg.data[0]
        self.p_des[1][0] = msg.data[1]

        self.p_des[2][0] = msg.data[2]
        self.p_des[3][0] = msg.data[3]

        self.p_des[4][0] = msg.data[4]
        self.p_des[5][0] = msg.data[5]

        self.p_des[6][0] = msg.data[6]
        self.p_des[7][0] = msg.data[7]


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




def main():
    # initialize a node
    rospy.init_node('level_frame_visualizer')

    # create instance of LevelFrameMapper class
    visualizer = LevelFrameVisualizer()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # OpenCV cleanup
    if visualizer.show:
        cv2.destroyAllWindows()

    # Save off the data file
    if visualizer.save_data:
        scipy.io.savemat(visualizer.outfile_str, mdict={'arr': visualizer.error_data})


if __name__ == '__main__':
    main()
