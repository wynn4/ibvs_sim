#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
import rospy
import numpy as np

class ArUcoAngle(object):

	def __init__(self):

		# Initialize class variables.
		self.R = np.eye(3, dtype=np.float32)

		# Initialize subscriber.
		self.aruco_est_sub = rospy.Subscriber('/aruco/estimate', PoseStamped, self.aruco_callback)


	def aruco_callback(self, msg):

		w = msg.pose.orientation.w
		x = msg.pose.orientation.x
		y = msg.pose.orientation.y
		z = msg.pose.orientation.z


		# Convert quaternion to a rotation.
		self.R = self.get_R_from_quaternion(w, x, y, z)

		vec = np.dot(self.R, np.array([[0], [0], [-1]]))

		# Find the angle between vec and the inertial k-axis.
		angle = np.arccos(np.dot(vec.T, np.array([[0], [0], [-1]])))

		print 'Angle: %f' % (180.0 - np.degrees(angle))

	def get_R_from_quaternion(self, w, x, y, z):
		
		wx = w*x
		wy = w*y
		wz = w*z
		xx = x*x
		xy = x*y
		xz = x*z
		yy = y*y
		yz = y*z
		zz = z*z

		return np.array([[1. - 2.*yy - 2.*zz, 2.*xy + 2.*wz, 2.*xz - 2.*wy],
			             [2.*xy - 2.*wz, 1. - 2.*xx - 2.*zz, 2.*yz + 2.*wx],
			             [2.*xz + 2.*wy, 2.*yz - 2.*wx, 1. - 2.*xx - 2.*yy]])








def main():
    # initialize a node
    rospy.init_node('aruco_angle')

    # create instance of ArUcoAngle class
    aruco_angle = ArUcoAngle()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__ == '__main__':
    main()

