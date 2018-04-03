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
# import time

## TODO: -finish initializing PID controllers in __init__  --done Jan 17
##       -add loading of applicable ros params in __init__ --done Jan 17
##       -finish compute_control function                  --done Jan 18
##       -finish saturate function                         --done Jan 18
##       -test it out and debug                            --done Jan 18
##       -add controller case for uvw commands (for IBVS)


class Controller(object):

    def __init__(self):

        # load ROS params

        # PID gains
        u_P = rospy.get_param('~u_P', 0.2)
        u_I = rospy.get_param('~u_I', 0.0)
        u_D = rospy.get_param('~u_D', 0.01)

        v_P = rospy.get_param('~v_P', 0.2)
        v_I = rospy.get_param('~v_I', 0.0)
        v_D = rospy.get_param('~v_D', 0.01)

        w_P = rospy.get_param('~w_P', 3.0)
        w_I = rospy.get_param('~w_I', 0.05)
        w_D = rospy.get_param('~w_D', 0.5)

        x_P = rospy.get_param('~x_P', 0.5)
        x_I = rospy.get_param('~x_I', 0.01)
        x_D = rospy.get_param('~x_D', 0.1)

        y_P = rospy.get_param('~y_P', 0.5)
        y_I = rospy.get_param('~y_I', 0.01)
        y_D = rospy.get_param('~y_D', 0.1)

        z_P = rospy.get_param('~z_P', 1.0)
        z_I = rospy.get_param('~z_I', 0.1)
        z_D = rospy.get_param('~z_D', 0.4)

        psi_P = rospy.get_param('~psi_P', 0.5)
        psi_I = rospy.get_param('~psi_I', 0.0)
        psi_D = rospy.get_param('~psi_D', 0.0)

        tau = rospy.get_param('~tau', 0.04)

        # quadcopter params
        self.max_thrust = rospy.get_param('dynamics/max_F', 60.0)
        self.mass = rospy.get_param('dynamics/mass', 3.0)
        self.thrust_eq = (9.80665 * self.mass) / self.max_thrust
        self.drag_constant = rospy.get_param('dynamics/linear_mu', 0.1)

        # initialize state variables
        self.pn = 0.0
        self.pe = 0.0
        self.pd = 0.0

        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        self.roll = 0.0
        self.pitch = 0.0
        self.yawrate = 0.0
        self.altitude = 0.0

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

        self.xc_phi = 0.0
        self.xc_theta = 0.0
        self.xc_psi = 0.0

        self.xc_u = 0.0
        self.xc_v = 0.0
        self.xc_pd = 0.0
        self.xc_r = 0.0

        self.xc_ax = 0.0
        self.xc_ay = 0.0
        self.xc_az = 0.0
        self.xc_r = 0.0

        self.xc_throttle = 0.0

        # initialize saturation values
        self.max_roll = rospy.get_param('~max_roll', 0.15)
        self.max_pitch = rospy.get_param('~max_pitch', 0.15)
        self.max_yaw_rate = rospy.get_param('~max_yaw_rate', np.radians(45.0))
        self.max_throttle = rospy.get_param('~max_throttle', 1.0)
        self.max_u = rospy.get_param('~max_u', 1.0)
        self.max_v = rospy.get_param('~max_v', 1.0)
        self.max_w = rospy.get_param('~max_x', 1.0)

        # initialize PID controllers
        self.PID_u = PID(u_P, u_I, u_D, None, None, tau)
        self.PID_v = PID(v_P, v_I, v_D, None, None, tau)
        self.PID_w = PID(w_P, w_I, w_D, None, None, tau)
        self.PID_x = PID(x_P, x_I, x_D, None, None, tau)
        self.PID_y = PID(y_P, y_I, y_D, None, None, tau)
        self.PID_z = PID(z_P, z_I, z_D, None, None, tau)
        self.PID_psi = PID(psi_P, psi_I, psi_D, None, None, tau)

        # initialize other class variables
        self.prev_time = 0.0
        self.is_flying = False
        self.control_mode = 4  # MODE_XPOS_YPOS_YAW_ALTITUDE
        self.ibvs_active = False

        self.command = Command()

        # dynamic reconfigure
        self.server = Server(ControllerConfig, self.reconfigure_callback)
        

        # initialize subscribers
        self.state_sub = rospy.Subscriber('estimate', Odometry, self.state_callback)
        self.is_flying_sub = rospy.Subscriber('is_flying', Bool, self.is_flying_callback)
        self.cmd_sub = rospy.Subscriber('high_level_command', Command, self.cmd_callback)
        self.ibvs_active_sub = rospy.Subscriber('ibvs_active', Bool, self.ibvs_active_callback)

        # initialize publishers
        self.command_pub = rospy.Publisher('command', Command, queue_size=10)

        # initialize timer
        self.update_rate = 200.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_command)

        self.command_count = 0
        self.update_rate2 = 1.0
        self.update_timer2 = rospy.Timer(rospy.Duration(1.0/self.update_rate2), self.change_command)


    def send_command(self, event):

        # t = time.time()  # for seeing how fast this runs

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

        # elapsed = time.time() - t
        # hz_approx = 1.0/elapsed
        # print(hz_approx)


    def change_command(self, event):

        # cyclical pattern of roll, pitch, yawrate, altitude commands
        self.command_count += 1

        if self.command_count > 0 and self.command_count <= 20:
            self.roll = 0.0
            self.pitch = 0.0
            self.yawrate = 0.0
            self.altitude = 2.0

        if self.command_count > 20 and self.command_count <= 30:
            self.roll = np.radians(10.0)
            self.pitch = np.radians(0.0)
            self.yawrate = np.radians(0.0)
            self.altitude = 2.0

        if self.command_count > 30 and self.command_count <= 40:
            self.roll = np.radians(-10.0)
            self.pitch = np.radians(-0.0)
            self.yawrate = np.radians(-0.0)
            self.altitude = 2.0

        # Reset
        if self.command_count == 41:
            self.command_count = 0


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


    def ibvs_active_callback(self, msg):

        self.ibvs_active = msg.data


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

        elif mode == Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE:
            self.xc_phi = msg.x
            self.xc_theta = msg.y
            self.xc_throttle = msg.F
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

        return config


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

            max_ax = np.sin(np.arccos(self.thrust_eq))
            max_ay = np.sin(np.arccos(self.thrust_eq))
            self.xc_ax = self.saturate(self.PID_u.computePID(self.xc_u, self.u, dt) + self.drag_constant*self.u / (9.80665 * self.mass), max_ax, -max_ax)
            self.xc_ay = self.saturate(self.PID_v.computePID(self.xc_v, self.v, dt) + self.drag_constant*self.v / (9.80665 * self.mass), max_ay, -max_ay)

            # nested loop for altitude
            pddot = -np.sin(self.theta) * self.u + np.sin(self.phi)*np.cos(self.theta)*self.v + np.cos(self.phi)*np.cos(self.theta)*self.w

            # check to see if IBVS is active
            if self.ibvs_active:
                pddot_c = self.saturate(self.xc_pd, self.max_w, -self.max_w)  # this term should be coming in as w and here we are assuming w is close enough to pddot
            else:
                pddot_c = self.saturate(self.PID_w.computePID(self.xc_pd, self.pd, dt, pddot), self.max_w, -self.max_w)

            self.xc_az = self.PID_z.computePID(pddot_c, pddot, dt)
            # print statement if you want
            mode_flag = Command.MODE_XACC_YACC_YAWRATE_AZ

        if mode_flag == Command.MODE_XACC_YACC_YAWRATE_AZ:

            # Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
            # This model tends to pop the MAV up in the air when a large change
            # in control is commanded as the MAV rotates to it's commanded attitude while also ramping up throttle.
            # It works quite well, but it is a little oversimplified.
            total_acc_c = np.sqrt((1.0-self.xc_az)*(1.0-self.xc_az) + self.xc_ax*self.xc_ax + self.xc_ay*self.xc_ay)  #(in g's)
            # print('roscopter/controller: total_acc = {}'.format(total_acc_c))
            if total_acc_c > 0.001:
                self.xc_phi = np.arcsin(self.xc_ay / total_acc_c)
                self.xc_theta = -1.0*np.arcsin(self.xc_ax / total_acc_c)
            else:
                self.xc_phi = 0.0
                self.xc_theta = 0.0

            # calculate actual throttle (saturate az to be falling at 1 g)
            max_az = 1.0 / self.thrust_eq
            self.xc_az = self.saturate(self.xc_az, 1.0, -max_az)
            total_acc_c = np.sqrt((1.0-self.xc_az)*(1.0-self.xc_az) + self.xc_ax*self.xc_ax + self.xc_ay*self.xc_ay)  #(in g's)
            self.xc_throttle = total_acc_c*self.thrust_eq  # calculate the total thrust in normalized units

            # print '\nxc_az:', self.xc_az, '\nmax_az:', max_az, '\ntotal_acc_c:', total_acc_c, '\nthrottle:', self.xc_throttle

            mode_flag = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE

        if mode_flag == Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE:

            # pack up and send the command
            self.command.mode = Command.MODE_ROLL_PITCH_YAWRATE_ALTITUDE
            # self.command.F = self.saturate(self.xc_throttle, self.max_throttle, 0.0)
            # self.command.x = self.saturate(self.xc_phi, self.max_roll, -self.max_roll)
            # self.command.y = self.saturate(self.xc_theta, self.max_pitch, -self.max_pitch)
            # self.command.z = self.saturate(self.xc_r, self.max_yaw_rate, -self.max_yaw_rate)

            # Note: In reality I've hijacked this mode in roscopter_sim/src/multirotor_forces_and_moments.cpp
            #       and used this as a place to implement the sliding-mode (SM) quadrotor controller.  The SM
            #       controller actually controls roll angle, pitch angle, yaw angle, and altitude and thus we
            #       fill out the command message as follows:
            self.command.F = -self.xc_pd
            self.command.x = self.saturate(self.xc_phi, self.max_roll, -self.max_roll)
            self.command.y = self.saturate(self.xc_theta, self.max_pitch, -self.max_pitch)
            self.command.z = self.xc_psi
        # self.command.mode = Command.MODE_ROLL_PITCH_YAWRATE_ALTITUDE
        # self.command.x = self.roll
        # self.command.y = self.pitch
        # self.command.z = self.yawrate
        # self.command.F = self.altitude



    def reset_integrators(self):

        self.PID_u.clearIntegrator()
        self.PID_v.clearIntegrator()
        self.PID_w.clearIntegrator()
        self.PID_x.clearIntegrator()
        self.PID_y.clearIntegrator()
        self.PID_z.clearIntegrator()
        self.PID_psi.clearIntegrator()

    
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
    rospy.init_node('controller')

    # create instance of Controller class
    controller = Controller()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__ == '__main__':
    main()
