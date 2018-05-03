#! /usr/bin/env python

## ROS node that publishes wind.
## JSW Jan 2018

import rospy
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import numpy as np
import tf


class Wind(object):

    def __init__(self):

        # Load ROS params.

        # Initialize other class variables.
        self.wind_N = rospy.get_param('~wind_N', 0.0)
        self.wind_E = rospy.get_param('~wind_E', 0.0)
        self.wind_D = rospy.get_param('~wind_D', 0.0)



        if np.sqrt(self.wind_N**2 + self.wind_E**2 + self.wind_D**2) > 0.0:
            self.wind = True
            self.wind_mag = np.sqrt(self.wind_N**2 + self.wind_E**2 + self.wind_D**2)
        else:
            self.wind = False
            self.wind_mag = 0.0

        self.max_gust = self.wind_mag / 2.0

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.h = 0.0
        
        self.R_b_v = np.zeros((3,3), dtype=np.float32)

        # Initialize timers.
        self.update_rate = 10.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.update_wind)

        # Initialize publisher
        self.wind_pub = rospy.Publisher("/quadcopter/wind", Vector3, queue_size=1)

        # Initialize subscriber
        self.state_sub = rospy.Subscriber('/quadcopter/ground_truth/odometry/NED', Odometry, self.state_callback)


    def update_wind(self, event):

        self.wind = False
        if self.wind:
            sphi = np.sin(self.phi)
            cphi = np.cos(self.phi)
            stheta = np.sin(self.theta)
            ctheta = np.cos(self.theta)
            spsi = np.sin(self.psi)
            cpsi = np.cos(self.psi)


            self.R_b_v = np.array([[ctheta*cpsi, ctheta*spsi, -stheta],
                                   [sphi*stheta*cpsi - cphi*spsi, sphi*stheta*spsi + cphi*cpsi, sphi*ctheta],
                                   [cphi*stheta*cpsi + sphi*spsi, cphi*stheta*spsi - sphi*cpsi, cphi*ctheta]]).T

            L_w = self.h
            L_u = self.h / ((0.177 + 0.000823*self.h)**1.2)
            L_v = L_u
            # print "L_u: " + str(L_u)
            # print "h: " + str(self.h)
            Sigma_w = 0.1 * self.wind_mag
            Sigma_u = Sigma_w / ((0.177 + 0.000823*self.h)**0.4)
            Sigma_v = Sigma_u

            s = np.random.random() * 2 - 1
            gust_u = Sigma_u * np.sqrt(L_u / np.pi*self.wind_mag) * ((1 + (np.sqrt(3.0)*L_u*s/self.wind_mag)) / (1 + (L_u/self.wind_mag)*s)**2)
            gust_u = self.saturate(gust_u)
            # print gust_u
            s = np.random.random() * 2 - 1
            gust_v = Sigma_v * np.sqrt(L_v / np.pi*self.wind_mag) * ((1 + (np.sqrt(3.0)*L_v*s/self.wind_mag)) / (1 + (L_v/self.wind_mag)*s)**2)
            gust_v = self.saturate(gust_v)
            s = np.random.random() * 2 - 1
            gust_w = Sigma_w * np.sqrt(L_w / np.pi*self.wind_mag) * ((1 + (np.sqrt(3.0)*L_w*s/self.wind_mag)) / (1 + (L_w/self.wind_mag)*s)**2)
            gust_w = self.saturate(gust_w)

            gust_vec_body = np.array([[gust_u], [gust_v], [gust_w]])

            # Rotate gust into the inertial (same as vehicle-1) frame
            gust_vec_vehicle = np.dot(self.R_b_v, gust_vec_body)

            wind_steady = np.array([[self.wind_N], [self.wind_E], [self.wind_D]])

            total_wind = wind_steady + gust_vec_vehicle
            # total_wind = gust_vec_vehicle
            # total_wind = np.zeros((3,1))

            # Publish.
            wind_msg = Vector3()
            wind_msg.x = total_wind[0][0]
            wind_msg.y = total_wind[1][0]
            wind_msg.z = total_wind[2][0]

            self.wind_pub.publish(wind_msg)

        else:
            wind_msg = Vector3()
            wind_msg.x = self.wind_N
            wind_msg.y = self.wind_E
            wind_msg.z = self.wind_D

            self.wind_pub.publish(wind_msg)


    def state_callback(self, msg):

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
        self.psi = euler[2]

        self.h = -msg.pose.pose.position.z

    def saturate(self, value):

        if value > self.max_gust:
            rval = self.max_gust
        elif value < -self.max_gust:
            rval = -self.max_gust
        else:
            rval = value
        return rval




    


def main():
    # initialize a node
    rospy.init_node('wind_publisher')

    # create instance of Wind class
    wind = Wind()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
