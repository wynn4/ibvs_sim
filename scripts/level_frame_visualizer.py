#! /usr/bin/env python

## Simple ROS node that subcribes to pixel data in the virtual level camera frame
## and displays a visualization of the pixel data
## JSW Jan 2018

import rospy
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry
from aruco_localization.msg import FloatList
import numpy as np
import cv2
import tf
import time


class LevelFrameVisualizer(object):

    def __init__(self):

        # load ROS params

        ## initialize other class variables

        # image size
        # TODO get these params automatically
        self.img_w = 1288
        self.img_h = 964

        # camera params
        self.K = np.zeros((3,3))
        self.d = np.zeros(5)
        self.fx = 0.0
        self.fy = 0.0
        self.cx = 0.0
        self.cy = 0.0
        self.f = 0.0

        # visualization params
        self.show = True
        shape = (self.img_h, self.img_w, 3)
        self.level_frame = np.zeros(shape, np.uint8)

        # desired pixel coords 
        # [u1, v1, u2, v2, u3, v3, u4, v4].T  8x1
        self.p_des = np.array([-200, -200, 200, -200, 200, 200, -200, 200], dtype=np.float32).reshape(8,1)

        # matrix to hold pixel data
        self.uv_bar_lf = np.zeros((4,2))  # pixel coords (u,v) of the corner points in the virtual level frame
        self.corners = np.zeros((4,2))

        # initialize subscribers
        self.uv_bar_sub = rospy.Subscriber('/ibvs/uv_bar_lf', FloatList, self.level_frame_corners_callback)
        self.corner_pix_sub = rospy.Subscriber('/aruco/marker_corners', FloatList, self.corners_callback)
        self.camera_info_sub = rospy.Subscriber('/quadcopter/camera/camera_info', CameraInfo, self.camera_info_callback)


    def level_frame_corners_callback(self, msg):

        # t = time.time()
        
        # populate the matrix of (u,v) pixel coordinates in the virtual-level-frame
        self.uv_bar_lf[0][0] = msg.data[0]   # u1
        self.uv_bar_lf[0][1] = msg.data[1]   # v1

        self.uv_bar_lf[1][0] = msg.data[2]   # u2
        self.uv_bar_lf[1][1] = msg.data[3]   # v2

        self.uv_bar_lf[2][0] = msg.data[4]   # u3
        self.uv_bar_lf[2][1] = msg.data[5]   # v3

        self.uv_bar_lf[3][0] = msg.data[6]   # u4
        self.uv_bar_lf[3][1] = msg.data[7]   # v4

        # print "data:"
        # print "uv_bar_lf: "
        # print(self.uv_bar_lf)
        # print "\n"

        # print "uv_bar_cam: "
        # print(corners_undist)
        # print "\n"

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

            # add some text labels
            cv2.putText(self.level_frame, "Camera Frame",(100,25),cv2.FONT_HERSHEY_PLAIN,2.0,(0,0,255),2)
            cv2.putText(self.level_frame, "Virtual Level Frame",(100,55),cv2.FONT_HERSHEY_PLAIN,2.0,(0,255,255),2)
            cv2.putText(self.level_frame, "Desired Pixel Coordinates (VLF)",(100,85),cv2.FONT_HERSHEY_PLAIN,2.0,(0,255,0),2)

            # display the image
            cv2.imshow("level_frame_image", self.level_frame)
            cv2.waitKey(1)

        # elapsed = time.time() - t
        # hz_approx = 1.0/elapsed
        # print(hz_approx)


    def corners_callback(self, msg):

        # populate corners matrix
        self.corners[0][0] = msg.data[0]
        self.corners[0][1] = msg.data[1]

        self.corners[1][0] = msg.data[2]
        self.corners[1][1] = msg.data[3]

        self.corners[2][0] = msg.data[4]
        self.corners[2][1] = msg.data[5]

        self.corners[3][0] = msg.data[6]
        self.corners[3][1] = msg.data[7]


    def camera_info_callback(self, msg):

        # get the Camera Matrix K and distortion params d
        self.K = np.array(msg.K, dtype=np.float32).reshape((3,3))
        self.d = np.array(msg.D, dtype=np.float32)

        self.fx = self.K[0][0]
        self.fy = self.K[1][1]
        self.cx = self.K[0][2]
        self.cy = self.K[1][2]

        self.f = (self.fx + self.fy) / 2.0

        # self.matrix[0][0] = self.fx
        # self.matrix[1][1] = self.fy

        # just get this data once
        self.camera_info_sub.unregister()
        print("Level_frame_visualizer: Got camera info!")




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

if __name__ == '__main__':
    main()
