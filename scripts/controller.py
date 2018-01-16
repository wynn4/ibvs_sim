#! /usr/bin/env python

## Python implementation of a quadcopter controller following the ROS Copter controller structure

import rospy
import numpy as np
from dynamic_reconfigure.server import Server
from ibvs_sim.cfg import ControllerConfig
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from rosflight_msgs.msg import Command
from geometry_msgs.msg import Pose
from simple_pid import PID
import tf

## TODO: -finish initializing PID controllers in __init__
##       -add loading of applicable ros params in __init__
##       -finish compute_control function
##       -finish saturate function
##       -test it out and debug
##       -add controller case for uvw commands (for IBVS)


class Controller(object):

    def __init__(self):

        # load ROS params

        # initialize state variables
        self.pn = 0.0
        self.pe = 0.0
        self.pd = 0.0

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.u = 0.0
        self.v = 0.0
        self.w = 0.0

        self.p = 0.0
        self.q = 0.0
        self.r = 0.0

        self.ax = 0.0
        self.ay = 0.0
        self.az = 0.0

        self.throttle = 0.0

        # initialize command variables
        self.xc_pn = 0.0
        self.xc_pe = 0.0
        self.xc_pd = 0.0
        self.xc_psi = 0.0

        self.xc_u = 0.0
        self.xc_v = 0.0
        self.xc_pd = 0.0
        self.xc_r = 0.0

        self.xc_ax = 0.0
        self.xc_ay = 0.0
        self.xc_az = 0.0
        self.xc_r = 0.0

        # initialize saturation values
        self.max_roll = rospy.get_param('max_roll', 0.15)
        self.max_pitch = rospy.get_param('max_pitch', 0.15)
        self.max_yaw_rate = rospy.get_param('max_yaw_rate', np.radians(45.0))
        self.max_throttle = rospy.get_param('max_throttle', 1.0)
        self.max_u = rospy.get_param('max_u', 1.0)
        self.max_v = rospy.get_param('max_v', 1.0)
        self.max_w = rospy.get_param('max_x', 1.0)

        # initialize PID controllers
        self.PID_u = PID()
        self.PID_v = PID()
        self.PID_w = PID()
        self.PID_x = PID()
        self.PID_y = PID()
        self.PID_z = PID()
        self.PID_psi = PID()

        # initialize other class variables
        self.prev_time = 0.0
        self.is_flying = False
        self.control_mode = 4  # MODE_XPOS_YPOS_YAW_ALTITUDE

        self.command = Command()

        # dynamic reconfigure
        self.server = Server(ControllerConfig, self.reconfigure_callback)
        

        # initialize subscribers
        self.state_sub = rospy.Subscriber('estimate', Odometry, self.state_callback)
        self.is_flying_sub = rospy.Subscriber('is_flying', Bool, self.is_flying_callback)
        self.cmd_sub = rospy.Subscriber('high_level_command', Command, self.cmd_callback)

        # initialize publishers
        self.command_pub = rospy.Publisher('command', Command, queue_size=10)

        # initialize timer
        self.update_rate = 200.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_command)


    def send_command(self, event):

        if self.prev_time == 0:
            self.prev_time = rospy.get_time()
            return

        # get dt
        now = rospy.get_time()
        dt = now - self.prev_time
        self.prev_time = now

        if dt <= 0:
            return

        if self.is_flying:
            self.compute_control(dt)
            self.command_pub.publish(self.command)
        else:
            reset_integrators()
            self.prev_time = rospy.get_time()


    def state_callback(self, msg):

        # this should already be coming in NED
        self.pn = msg.pose.pose.position.x
        self.pe = msg.pose.pose.position.y
        self.pd = msg.pose.pose.position.z

        self.u = msg.twist.twist.linear.x
        self.v = msg.twist.twist.linear.y
        self.w = msg.twist.twist.linear.z

        # convert quaternion to RPY
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.phi = euler[0]
        self.theta = euler[1]
        self.psi = euler[2]

        self.p = msg.twist.twist.angular.x
        self.q = msg.twist.twist.angular.y
        self.r = msg.twist.twist.angular.z


    def is_flying_callback(self, msg):

        self.is_flying = msg.data


    def cmd_callback(self, msg):

        mode = msg.mode

        if mode == Command.MODE_XPOS_YPOS_YAW_ALTITUDE:
            self.xc_pn = msg.x
            self.xc_pe = msg.y
            self.xc_pd = msg.F
            self.xc_psi = msg.z
            self.control_mode = mode

        elif mode == Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
            self.xc_u = msg.x
            self.xc_v = msg.y
            self.xc_pd = msg.F
            self.xc_r = msg.z
            self.control_mode = mode

        elif mode == Command.MODE_XACC_YACC_YAWRATE_AZ:
            self.xc_ax = msg.x
            self.xc_ay = msg.y
            self.xc_az = msg.F
            self.xc_r = msg.z
            self.control_mode = mode

        else:
            print('roscopter/controller: Unhandled command message of type {}'.format(mode))


    def reconfigure_callback(self, config, level):

        tau = config.tau
        P = config.u_P
        I = config.u_I
        D = config.u_D
        self.PID_u.setGains(P, I, D, tau)

        P = config.v_P
        I = config.v_I
        D = config.v_D
        self.PID_v.setGains(P, I, D, tau)

        P = config.w_P
        I = config.w_I
        D = config.w_D
        self.PID_w.setGains(P, I, D, tau)

        P = config.x_P
        I = config.x_I
        D = config.x_D
        self.PID_x.setGains(P, I, D, tau)

        P = config.y_P
        I = config.y_I
        D = config.y_D
        self.PID_y.setGains(P, I, D, tau)

        P = config.z_P
        I = config.z_I
        D = config.z_D
        self.PID_z.setGains(P, I, D, tau)

        P = config.psi_P
        I = config.psi_I
        D = config.psi_D
        self.PID_psi.setGains(P, I, D, tau)

        self.max_roll = config.max_roll
        self.max_pitch = config.max_pitch
        self.max_yaw_rate = config.max_yaw_rate
        self.max_throttle = config.max_throttle
        self.max_u = config.max_u
        self.max_v = config.max_v
        self.max_w = config.max_w

        print('roscopter/controller: new gains')

        self.reset_integrators()


    def compute_control(self, dt):

        if dt <= 0.0000001:  # messes up derivative calculation in PID controllers
            return

        mode_flag = self.control_mode

        if mode_flag == Command.MODE_XPOS_YPOS_YAW_ALTITUDE:

            # figure out desired velocities (in inertial frame)
            # by running the position controllers
            pndot_c = self.PID_x.computePID(self.xc_pn, self.pn, dt)
            pedot_c = self.PID_y.computePID(self.xc_pe, self.pe, dt)

            # calculate desired yaw rate
            # first, determine the shortest direction to the commanded psi
            if abs(self.xc_psi + 2.0*np.pi - self.psi) < abs(self.xc_psi - self.psi):
                self.xc_psi += 2.0 * np.pi
            elif abs(self.xc_psi - 2.0*np.pi - self.psi) < abs(self.xc_psi - self.psi):
                self.xc_psi -= 2.0 * np.pi

            self.xc_r = self.saturate(self.PID_psi.computePID(self.xc_psi, self.psi, dt), self.max_yaw_rate, -self.max_yaw_rate)

            # rotate into body frame
            # TODO: include pitch and role in this mapping
            self.xc_u = self.saturate(pndot_c * np.cos(self.psi) + pedot_c * np.sin(self.psi), self.max_u, -self.max_u)
            self.xc_v = self.saturate(-pndot_c * np.sin(self.psi) + pedot_c * np.cos(self.psi), self.max_v, -self.max_v)

            mode_flag = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE

        if mode_flag == Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE:

            # stuff

        if mode_flag == Command.MODE_XACC_YACC_YAWRATE_AZ:

            # stuff

        if mode_flag == Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE:

            # stuff


        





    def reset_integrators(self):

        self.PID_u.clearIntegrator()
        self.PID_v.clearIntegrator()
        self.PID_w.clearIntegrator()
        self.PID_x.clearIntegrator()
        self.PID_y.clearIntegrator()
        self.PID_z.clearIntegrator()
        self.PID_psi.clearIntegrator()

    
    def saturate(self):

        # stuff




def main():
    # initialize a node
    rospy.init_node('controller')

    # create instance of LevelFrameMapper class
    controller = Controller()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__ == '__main__':
    main()
