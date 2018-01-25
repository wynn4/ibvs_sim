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
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import tf
import time


class ImageBasedVisualServoing(object):

    def __init__(self):

        # load ROS params

        ## initialize other class variables

        # if set to True, IBVS is computed according to eq. 15.11 from Corke
        # if set to False, IBVS is computed according to eq. 10 from Lee
        self.inverse_method = True

        # IBVS saturation values
        self.u_max = 0.2
        self.v_max = 0.2
        self.w_max = 0.7
        self.psidot_max = np.radians(45.0)

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
        self.f = 1000.0  # initialize to ge greater than zero

        pixel_um = 3.75  # pixel size in micrometers
        pixel_size = pixel_um * 1e-6  # pixel size in meters

        # copter attitude roll and pitch
        # self.phi = 0.0
        # self.theta = 0.0
        self.altitude = 0.0

        # distance (height) to the ArUco
        self.z_c = 10.0  # initialize to be greater than zero

        # positive definite weighting matrix W from Lee eq. 10 (tuning param)
        self.W = pixel_size * np.diag([0.1, 0.1, 10., 1., 1., 1.])

        # lambda from Corke eq. 15.11 (turning param)
        self.lam = np.array([2.0, 2.0, 1.0, 1.0, 1.0, 1.0], dtype=np.float32).reshape(6,1)  # 6x1

        # desired pixel coords 
        # [u1, v1, u2, v2, u3, v3, u4, v4].T  8x1
        # self.p_des = np.array([-200, -200, 200, -200, 200, 200, -200, 200], dtype=np.float32).reshape(8,1)  # 8x1
        self.p_des = np.array([0, -100, 100, 0, 0, 100, -100, 0], dtype=np.float32).reshape(8,1)  # 8x1

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
        self.uv_bar_sub = rospy.Subscriber('/ibvs/uv_bar_lf', FloatList, self.level_frame_corners_callback)
        self.aruco_sub = rospy.Subscriber('/aruco/estimate', PoseStamped, self.aruco_callback)
        self.attitude_sub = rospy.Subscriber('/quadcopter/estimate', Odometry, self.altitude_callback)
        self.camera_info_sub = rospy.Subscriber('/quadcopter/camera/camera_info', CameraInfo, self.camera_info_callback)

        # initialize publishers
        self.vel_cmd_pub = rospy.Publisher('/ibvs/vel_cmd', Twist, queue_size=1)


    def level_frame_corners_callback(self, msg):

        # t = time.time()

        # formulate image Jacobians according to eq(5)
        u1 = msg.data[0]
        v1 = msg.data[1]
        Jp1 = np.array([[-self.f/self.z_c, 0.0, u1/self.z_c, u1*v1/self.f, -(self.f**2 + u1**2)/self.f, v1],
                        [0.0, -self.f/self.z_c, v1/self.z_c, (self.f**2 + v1**2)/self.f, -u1*v1/self.f, -u1]])  # 2x6
        
        u2 = msg.data[2]
        v2 = msg.data[3]
        Jp2 = np.array([[-self.f/self.z_c, 0.0, u2/self.z_c, u2*v2/self.f, -(self.f**2 + u2**2)/self.f, v2],
                        [0.0, -self.f/self.z_c, v2/self.z_c, (self.f**2 + v2**2)/self.f, -u2*v2/self.f, -u2]])  # 2x6

        u3 = msg.data[4]
        v3 = msg.data[5]
        Jp3 = np.array([[-self.f/self.z_c, 0.0, u3/self.z_c, u3*v3/self.f, -(self.f**2 + u3**2)/self.f, v3],
                        [0.0, -self.f/self.z_c, v3/self.z_c, (self.f**2 + v3**2)/self.f, -u3*v3/self.f, -u3]])  # 2x6

        u4 = msg.data[6]
        v4 = msg.data[7]
        Jp4 = np.array([[-self.f/self.z_c, 0.0, u4/self.z_c, u4*v4/self.f, -(self.f**2 + u4**2)/self.f, v4],
                        [0.0, -self.f/self.z_c, v4/self.z_c, (self.f**2 + v4**2)/self.f, -u4*v4/self.f, -u4]])  # 2x6

        # stack the Jacobians
        Jp = np.vstack((Jp1, Jp2, Jp3, Jp4))  # 8x6

        # formulate the error term e = p - p_des
        p = np.array([u1, v1, u2, v2, u3, v3, u4, v4]).reshape(8,1)
        # e = p - self.p_des  # 8x1

        # formulate the desired camera velocity vector rdot_des
        if self.inverse_method:
            e = self.p_des - p
            # v = lambda * pinv(Jp) * e
            rdot_des = self.lam * np.linalg.pinv(Jp).dot(e)
            # NOTE: In the future, I may want to try Corke's 2nd Order Jacobian (eq. 15.12)
        else:
            e = p - self.p_des
            # rdot_des = -self.W * Jp.T * e
            rdot_des = - self.W.dot(Jp.T).dot(e)  # Lee eq. 10

        # break rdot_des into its linear and angular components
        rdot_des_linear = rdot_des[:3][:]  # 3x1 [vx, vy, vz].T
        rdot_des_angular = rdot_des[3:][:]  # 3x1 [wx, wy, wz].T

        # rotate rdot_des from the virtual level camera frame into the vehicle 1 frame
        # TODO maybe we should rotate all the way into the body frame???
        v_des_v1_linear = np.dot(self.R_vlc_v1, rdot_des_linear)  # 3x1
        v_des_v1_angular = np.dot(self.R_vlc_v1, rdot_des_angular)  # 3x1

        # print "\nv_des_linear: ", '\n', v_des_v1_linear
        
        # saturate and fill out the velocity command message
        self.vel_cmd_msg.linear.x = self.saturate(v_des_v1_linear[0][0], self.u_max, -self.u_max)
        self.vel_cmd_msg.linear.y = self.saturate(v_des_v1_linear[1][0], self.v_max, -self.v_max)
        self.vel_cmd_msg.linear.z = self.saturate(v_des_v1_linear[2][0], self.w_max, -self.w_max)

        self.vel_cmd_msg.angular.x = v_des_v1_angular[0][0]
        self.vel_cmd_msg.angular.y = v_des_v1_angular[1][0]
        self.vel_cmd_msg.angular.z = self.saturate(v_des_v1_angular[2][0], self.psidot_max, -self.psidot_max)

        # publish
        self.vel_cmd_pub.publish(self.vel_cmd_msg)

        
    def altitude_callback(self, msg):

        # get the quaternion orientation from the message
        # quaternion = (
        #     msg.pose.pose.orientation.x,
        #     msg.pose.pose.orientation.y,
        #     msg.pose.pose.orientation.z,
        #     msg.pose.pose.orientation.w)

        # # convert to euler angles 
        # euler = tf.transformations.euler_from_quaternion(quaternion)

        # # update class variables
        # self.phi = euler[0]
        # self.theta = euler[1]
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

        # just get this data once
        self.camera_info_sub.unregister()
        print("IBVS: Got camera info!")


    def aruco_callback(self, msg):

        # check for NaNs coming from ArUco estimate
        if np.isnan(msg.pose.position.z):
            # print "Nan!"
            z_c = self.altitude

            if z_c >= 0.5 and self.inverse_method == False:
                self.z_c = 0.5
            else:
               self.z_c = z_c

            return


        
        # all we need is distance to the ArUco, z_c
        z_c = msg.pose.position.z

        if z_c >= 0.5 and self.inverse_method == False:
            self.z_c = 0.5
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
