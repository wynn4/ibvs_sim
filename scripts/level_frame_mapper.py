#! /usr/bin/env python

## Simple ROS node that subcribes to pixel data in the fixed camera frame and
## transforms it to be expressed in virtual level camera frame
## JSW Nov 2017

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
import dynamic_reconfigure.client
import cv2
import math
import numpy as np


class LevelFrameMapper(object):

    def __init__(self):

        # load ROS params

        # initialize other class variables

        # initialize variables to be published

        # initialize CVBridge object

        # initialize subscriber


        # initialize publishers

        # initialize timer
        # self.exposure_update_rate = 1.0
        # self.update_timer = rospy.Timer(rospy.Duration(1.0/self.exposure_update_rate), self.process_image)


    def callback(self, msg):




def main():
    # initialize a node
    rospy.init_node('level_frame_pixel_publisher')

    # create instance of LevelFrameMapper class
    publisher = LevelFrameMapper()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

    # OpenCV cleanup
    # cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
