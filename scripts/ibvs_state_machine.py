#!/usr/bin/env python

## Simple state machine implemented in ROS that manages whether a PX4 copter
## follows position commands or IBVS velocity commands.
## JSW Mar 2018

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from geometry_msgs.msg import Point32
from geometry_msgs.msg import PoseStamped
from rosflight_msgs.msg import Command
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import AttitudeTarget
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
import numpy as np
import tf
from collections import deque



# TODO:
# - Add wind-compensated rendezvous point -- Done April 6, 2018
# - Don't switch into IBVS until you've reached the rendezvous point -- Done April 6, 2018
# - Handle switching to the inner marker corners better -- Done May 24
#    - The count method isn't great
# - Touchdown timing
#    -don't land when the aruco is rolled or pitched a lot -- Done May 9, 2018
# - Resetting of state machine when ArUco is lost
# - Bread-crumb trail
# - Open-loop search for ArUco (maybe)

# Flight Tested May 31 and it works for single and nested marker



class StateMachine():

    def __init__(self):

        # load parameters from param server

        # Set flag for interfacing with ROScopter or MAVROS
        self.mode_flag = rospy.get_param('~mode', 'mavros')

        # Initialize status flags
        self.status_flag = 'RENDEZVOUS'
        self.prev_status = 'MISSION'
        self.status_flag_msg = String()
        self.status_flag_msg.data = self.status_flag
        self.ibvs_status_flag_msg = String()

        # Initialize current target flag
        self.current_target = 'aruco_outer'

        # Initialize visibility flags
        self.outer_target_is_visible = False
        self.inner_target_is_visible = False
        self.current_target_is_visible = False
        self.other_target_is_visible = False



        # Velocity saturation values
        self.u_max = rospy.get_param('~u_max', 0.5)
        self.v_max = rospy.get_param('~v_max', 0.5)
        self.w_max = rospy.get_param('~w_max', 0.7)

        self.u_max_inner = rospy.get_param('~u_max_inner', 0.2)
        self.v_max_inner = rospy.get_param('~v_max_inner', 0.2)
        self.w_max_inner = rospy.get_param('~w_max_inner', 0.2)

        self.rendezvous_height = rospy.get_param('~rendezvous_height', 10.0)
        self.wp_threshold = rospy.get_param('~wp_threshold', 1.0)

        self.landing_distance_threshold = rospy.get_param('~landing_distance_threshold', 0.4)
        self.p_des_error_outer_threshold = rospy.get_param('~p_des_error_outer_threshold', 50.0)
        self.p_des_error_inner_threshold = rospy.get_param('~p_des_error_inner_threshold', 50.0)

        self.inner_error_condition = rospy.get_param('~inner_error_condition', False)

        self.test_name = rospy.get_param('~test_name', 'test1')

        self.landing_thrust0 = rospy.get_param('~landing_thrust', 0.45)
        self.thrust_val = self.landing_thrust0

        self.wp_error = 1.0e3
        self.p_des_error_outer = 1.0e3
        self.p_des_error_inner = 1.0e3

        # initialize target location and velocity
        self.target_N = 0.0
        self.target_E = 0.0
        self.target_VN = 0.0
        self.target_VE = 0.0

        # Initialize waypoint setpoint
        self.wp_N = 5.0
        self.wp_E = 5.0
        self.wp_D = -self.rendezvous_height
        self.heading_command = np.radians(0.0)

        self.relative_heading = 0.0

        self.wind_calc_completed = rospy.get_param('~wind_calc_completed', False)
        self.wind_window_seconds = 5
        self.wind_calc_duration = 10.0
        self.wind_offset = np.zeros((3,1), dtype=np.float32)

        self.heading_correction_completed = rospy.get_param('~heading_correction_completed', False)

        # Initialize queues
        if self.mode_flag == 'mavros':
            # this assumes state estimates at ~60 hz which gives us 5 seconds of data
            self.queue_length = 60*self.wind_window_seconds
        else:
            # this assumes we're in roscopter mode and state comes in at ~170 hz
            self.queue_length = 170*self.wind_window_seconds

        self.phi_queue = deque(maxlen=self.queue_length)
        self.theta_queue = deque(maxlen=self.queue_length)

        # Average roll and pitch values in the wind
        self.roll_avg = 0.0
        self.pitch_avg = 0.0

        # Rotation matrix to hold ArUco attitude data
        self.R_aruco = np.eye(3, dtype=np.float32)

        self.max_boat_angle = np.radians(15.0)

        # Flag for wheter or not is is a good time to land
        self.safe_to_land = False

        # Initialize state variables
        self.pn = 0.0
        self.pe = 0.0
        self.pd = 0.0
        self.phi = 0.0
        self.theta = 0.0
        self.psi = 0.0

        # ibvs parameters
        # outer
        self.ibvs_x = 0.0
        self.ibvs_y = 0.0
        self.ibvs_F = 0.0
        self.ibvs_z = 0.0

        self.visibility_queue_length = 5
        self.ibvs_count = 0
        self.ibvs_time_outer = rospy.get_time() - 100.0  # In the past
        self.ibvs_time_queue_outer = deque(maxlen=self.visibility_queue_length)

        # inner
        self.ibvs_x_inner = 0.0
        self.ibvs_y_inner = 0.0
        self.ibvs_F_inner = 0.0
        self.ibvs_z_inner = 0.0

        self.ibvs_count_inner = 0
        self.ibvs_time_inner = rospy.get_time() - 100.0  # In the past
        self.ibvs_time_queue_inner = deque(maxlen=self.visibility_queue_length)

        # Fill up the queues with 'old' timestamps
        for x in range(0,self.visibility_queue_length):
            self.ibvs_time_queue_outer.appendleft(self.ibvs_time_outer)
            self.ibvs_time_queue_inner.appendleft(self.ibvs_time_inner)

        self.distance = 10.0

        # self.land_mode_sent = False

        self.ned_vel_vec_inner = np.array([[0.0],
                                           [0.0],
                                           [0.0]], dtype=np.float32)

        self.ned_vel_vec_outer = np.array([[0.0],
                                           [0.0],
                                           [0.0]], dtype=np.float32)

        # Initialize rotation from NED to ENU (for MAVROS).
        self.R_ned_enu = np.array([[0., 1., 0.],
                                   [1., 0., 0.],
                                   [0., 0., -1.]])

        # Create a mask of PositionTarget msg fields we wish to ignore.
        # Here we have 'OR'ed together all of the fields we don't want
        # (i.e., we want x-vel, y-vel, z-vel, yawrate).
        self.velocity_mask = (PositionTarget.IGNORE_PX
                            | PositionTarget.IGNORE_PY
                            | PositionTarget.IGNORE_PZ
                            | PositionTarget.IGNORE_AFX
                            | PositionTarget.IGNORE_AFY
                            | PositionTarget.IGNORE_AFZ
                            | PositionTarget.FORCE
                            | PositionTarget.IGNORE_YAW)

        self.ned_pos_mask = (PositionTarget.IGNORE_VX
                            | PositionTarget.IGNORE_VY
                            | PositionTarget.IGNORE_VZ
                            | PositionTarget.IGNORE_AFX
                            | PositionTarget.IGNORE_AFY
                            | PositionTarget.IGNORE_AFZ
                            | PositionTarget.FORCE
                            | PositionTarget.IGNORE_YAW_RATE)

        self.attitude_mask = (AttitudeTarget.IGNORE_ATTITUDE)

        # Set Up Publishers and Subscribers
        self.target_sub = rospy.Subscriber('/target_position', Odometry, self.target_callback, queue_size=1)
        self.ibvs_sub = rospy.Subscriber('/ibvs/vel_cmd', Twist, self.ibvs_velocity_cmd_callback, queue_size=1)
        self.ibvs_inner_sub = rospy.Subscriber('/ibvs_inner/vel_cmd', Twist, self.ibvs_velocity_cmd_inner_callback, queue_size=1)
        self.aruco_sub = rospy.Subscriber('/aruco/distance_inner', Float32, self.aruco_inner_distance_callback)
        self.aruco_att_sub = rospy.Subscriber('/aruco/estimate', PoseStamped, self.aruco_att_callback)
        self.aruco_heading_sub = rospy.Subscriber('/aruco/heading_outer', Float32, self.aruco_relative_heading_callback)
        self.state_sub = rospy.Subscriber('estimate', Odometry, self.state_callback)
        self.ibvs_ave_error_sub = rospy.Subscriber('/ibvs/ibvs_error_outer', Float32, self.ibvs_ave_error_callback)
        self.ibvs_ave_error_inner_sub = rospy.Subscriber('/ibvs/ibvs_error_inner', Float32, self.ibvs_ave_error_inner_callback)
        self.target_velocity_sub = rospy.Subscriber('/target_ekf/velocity_lpf', Point32, self.target_velocity_callback)


        self.command_pub_roscopter = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)
        self.command_pub_mavros = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.attitude_pub_mavros = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=1)
        self.avg_attitude_pub = rospy.Publisher('/quadcopter/attitude_avg', Point, queue_size=1)
        self.ibvs_active_pub_ = rospy.Publisher('/quadcopter/ibvs_active', Bool, queue_size=1)
        self.status_flag_pub = rospy.Publisher('/status_flag', String, queue_size=1)
        self.ibvs_status_flag_pub = rospy.Publisher('/ibvs_status_flag', String, queue_size=1)

        # Set Up Service Proxy
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.arm_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

        self.ibvs_active_msg = Bool()
        self.ibvs_active_msg.data = False

        # Initialize timers.
        self.avg_attitude_rate = 1.0
        self.avg_attitude_timer = rospy.Timer(rospy.Duration(1.0/self.avg_attitude_rate), self.send_avg_attitude)

        # self.status_update_rate = 5.0
        # self.status_timer = rospy.Timer(rospy.Duration(1.0/self.status_update_rate), self.update_status)

        self.command_update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.command_update_rate), self.send_commands)


    def send_commands(self, event):

        # then = rospy.get_time()
        self.update_marker_visibility_status()

        self.update_state_machine_status_and_send_command()
        # now = rospy.get_time()
        # secs = now - then
        # print 'Loop seconds: %f' % secs


    def update_state_machine_status_and_send_command(self):
        # print self.status_flag

        self.update_wp_error()

        if self.status_flag == 'RENDEZVOUS':

            # Is a target in view?
            if self.outer_target_is_visible or self.inner_target_is_visible:

                # Were we just in IBVS mode?
                if self.prev_status == 'IBVS':

                    # Go back into IBVS
                    self.status_flag = 'IBVS'
                    self.prev_status = 'RENDEZVOUS'

        # RENDEZVOUS MODE
        if self.status_flag == 'RENDEZVOUS':

            # Is waypoint error sufficiently small?
            if self.wp_error <= self.wp_threshold:

                # Has Wind Calibration occured yet?
                if self.wind_calc_completed == False:

                    # Switch to wind calibration status
                    self.status_flag = 'WIND_CALIBRATION'
                    self.prev_status = 'RENDEZVOUS'
                    self.wind_calc_time = rospy.get_time()


                elif self.wind_calc_completed == True and self.heading_correction_completed == False:
                    
                    # Perform initial heading correction
                    self.status_flag = 'HEADING_CORRECTION'
                    self.prev_status = 'RENDEZVOUS'

                else:

                    # Is the target in view?
                    if self.outer_target_is_visible:

                        # Enter IBVS Mode
                        self.status_flag = 'IBVS'
                        self.prev_status = 'RENDEZVOUS'

                    else:

                        print "Fail. Returning to mode RENDEZVOUS"


            else:
                self.status_flag = 'RENDEZVOUS'


        # WIND CALIBRATION MODE
        if self.status_flag == 'WIND_CALIBRATION':

            now = rospy.get_time()
            if now - self.wind_calc_time > self.wind_calc_duration:
                self.roll_avg = sum(self.phi_queue)/len(self.phi_queue)
                self.pitch_avg = sum(self.theta_queue)/len(self.theta_queue)
                self.wind_offset = self.compute_rendezvous_offset(self.roll_avg, self.pitch_avg)
                self.wp_N = self.target_N + self.wind_offset[0][0]
                self.wp_E = self.target_E + self.wind_offset[1][0]
                self.wind_calc_completed = True
                self.status_flag = 'RENDEZVOUS'
                self.prev_status = 'WIND_CALIBRATION'
                print "Average roll angle: %f \nAverage pitch angle: %f" % (np.degrees(self.roll_avg), np.degrees(self.pitch_avg))
                self.write_ave_att_file(self.roll_avg, self.pitch_avg)

        
        # HEADING CORRECTION MODE
        if self.status_flag == 'HEADING_CORRECTION':

            if self.heading_correction_completed == False:
                des_heading = self.psi + self.relative_heading
                while des_heading > np.radians(180.0):  des_heading = des_heading - np.radians(360.0)
                while des_heading < np.radians(-180.0):  des_heading = des_heading + np.radians(360.0)
                self.heading_command = des_heading

                # Update average roll and pitch angles after the yaw manuver (rotate about z-axis by relative heading)
                psi = self.relative_heading
                R = np.array([[np.cos(psi), -np.sin(psi)],[np.sin(psi), np.cos(psi)]]).T
                roll_pitch = np.dot(R, np.array([[self.roll_avg],[self.pitch_avg]]))
                self.roll_avg = roll_pitch[0][0]
                self.pitch_avg = roll_pitch[1][0]

                # print "Average roll angle: %f \nAverage pitch angle: %f" % (np.degrees(self.roll_avg), np.degrees(self.pitch_avg))

                self.heading_correction_completed = True
            if abs(self.relative_heading) <= np.radians(30.0):
                self.status_flag = 'RENDEZVOUS'
                self.prev_status = 'HEADING_CORRECTION'

        
        # IMAGE-BASED VISUAL SERVOING MODE
        if self.status_flag == 'IBVS':

            self.ibvs_active_msg.data = True
            self.ibvs_active_pub_.publish(self.ibvs_active_msg)
            self.enter_ibvs_state_machine()

        # WAYPOINT MODE (RENDEZVOUS, etc.)
        elif self.status_flag == 'RENDEZVOUS' or self.status_flag == 'WIND_CALIBRATION' or self.status_flag == 'HEADING_CORRECTION':

            self.ibvs_active_msg.data = False
            self.ibvs_active_pub_.publish(self.ibvs_active_msg)
            self.send_waypoint_command()

        # LAND MODE
        else:

            self.ibvs_active_msg.data = False
            self.ibvs_active_pub_.publish(self.ibvs_active_msg)

            # if self.mode_flag == 'mavros':
            #     # We use the AttitudeTarget message to do the final landing manuver.
            #     self.send_attitude_command()


        self.status_flag_msg.data = self.status_flag
        self.status_flag_pub.publish(self.status_flag_msg)

        self.ibvs_status_flag_msg.data = self.current_target
        self.ibvs_status_flag_pub.publish(self.ibvs_status_flag_msg)



    def enter_ibvs_state_machine(self):

        # Is the current target still visible?
        if self.current_target_is_visible:

            if self.current_target == 'aruco_outer':

                # Is the inner target visible?
                if self.inner_target_is_visible:

                    # Has the outer target been driven to its desired location?
                    if self.p_des_error_outer <= self.p_des_error_outer_threshold:

                        # Switch to controlling off of the inner ArUco
                        self.execute_ibvs('inner')
                    else:

                        # Execute IBVS(outer)
                        self.execute_ibvs('outer')

                else:

                    # Execute IBVS(outer)
                    self.execute_ibvs('outer')

            elif self.current_target == 'aruco_inner':

                # Execute IBVS(inner)
                self.execute_ibvs('inner')

            else:

                print "State Machine: Invalid current_target flag."

        else:

            # Is the other target visible?
            if self.other_target_is_visible:

                # Execute IBVS(other)
                self.execute_ibvs('other')

            else:

                # Return to mode RENDEZVOUS
                self.status_flag = 'RENDEZVOUS'
                self.prev_status = 'IBVS'

                # Maybe should set current_target = 'aruco_outer' here


    def execute_ibvs(self, target_flag):

        # Is target flag == 'outer'
        if target_flag == 'outer':

            # Set the current target flag
            self.current_target = 'aruco_outer'

            # Send the command
            self.send_ibvs_command('outer')

        # Is target flag == 'inner'
        elif target_flag == 'inner':

            # Is the distance to the ArUco sufficiently small?
            if self.distance <= self.landing_distance_threshold:

                # Is the target sufficiently level?
                if self.safe_to_land:

                    # Execute Landing!!!
                    self.execute_landing()

                else:
                    # Set the current target flag
                    self.current_target = 'aruco_inner'

                    # Send the command
                    self.send_ibvs_command('inner')

            else:

                # Set the current target flag
                self.current_target = 'aruco_inner'

                # Send the command
                self.send_ibvs_command('inner')

        # Target flag == 'other'
        else:

            # Send other ibvs
            if self.current_target == 'aruco_outer':

                self.current_target = 'aruco_inner'
                self.send_ibvs_command('inner')

            else:

                self.current_target = 'aruco_outer'
                self.send_ibvs_command('outer')


    def update_marker_visibility_status(self):

        now = rospy.get_time()

        if now - self.ibvs_time_queue_outer[-1] <= 1.0:
            self.outer_target_is_visible = True
        else:
            self.outer_target_is_visible = False

        if now - self.ibvs_time_queue_inner[-1] <= 1.0:
            self.inner_target_is_visible = True
        else:
            self.inner_target_is_visible = False

        # If current_target == 'aruco_outer'
        if self.current_target == 'aruco_outer':

            if self.outer_target_is_visible:

                self.current_target_is_visible = True

                if self.inner_target_is_visible:

                    self.other_target_is_visible = True

                else:

                    self.other_target_is_visible = False

            else:

                self.current_target_is_visible = False

                if self.inner_target_is_visible:

                    self.other_target_is_visible = True

                else:

                    self.other_target_is_visible = False

        
        # If current_target == 'aruco_inner'
        elif self.current_target == 'aruco_inner':

            if self.inner_target_is_visible:

                self.current_target_is_visible = True

                if self.outer_target_is_visible:

                    self.other_target_is_visible = True

                else:

                    self.other_target_is_visible = False

            else:

                self.current_target_is_visible = False

                if self.outer_target_is_visible:

                    self.other_target_is_visible = True

                else:

                    self.other_target_is_visible = False

        else:

            print 'State Machine: Invalid target flag.'


    def execute_landing(self):
        
        if self.mode_flag == 'mavros':

            # Update the status flag.
            self.status_flag = 'LAND'

            # Ramp down the motors.
            self.ramp_down_motors(self.landing_thrust0)

            
            # rospy.wait_for_service('/mavros/set_mode')
            # try:
            #     isModeChanged = self.set_mode_srv(custom_mode='AUTO.LAND')
            
            #     if isModeChanged:
            #         self.land_mode_sent = True
            #         print "mode %s sent." % 'AUTO.LAND'
            #         self.status_flag = 'LAND'

            # except rospy.ServiceException, e:
            #     print "service call set_mode failed: %s" % e

        else:
            
            command_msg = Command()
            command_msg.x = 0.0
            command_msg.y = 0.0
            command_msg.F = 0.0
            command_msg.z = 0.0
            command_msg.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
            self.command_pub_roscopter.publish(command_msg)
            self.status_flag = 'LAND'


    def ramp_down_motors(self, start_thrust):

        self.thrust_val = start_thrust

        # We want the ramp down to take place in 1.0 seconds
        ramp_time = 1.0
        decrement = (start_thrust - 0.2) / 10.0

        for x in range(0, 10):

            self.thrust_val -= decrement
            # print self.thrust_val
            # print "there"

            # Send the attitude and thrust setpoint message.
            self.send_attitude_command()

            # Wait a sec since we want the ramp down to happen in 0.5 seconds.
            rospy.sleep(ramp_time / 10.0)

        # Disarm Here
        rospy.sleep(0.05)
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            success = self.arm_srv(value=False)

            if success:
                print "Disarm."
        except rospy.ServiceException, e:
                print "service call disarm failed: %s" % e


    def send_ibvs_command(self, flag):

        if self.mode_flag == 'mavros':
            if flag == 'inner':

                ibvs_command_msg = PositionTarget()
                ibvs_command_msg.header.stamp = rospy.get_rostime()
                ibvs_command_msg.coordinate_frame = ibvs_command_msg.FRAME_BODY_NED
                ibvs_command_msg.type_mask = self.velocity_mask

                ibvs_command_msg.velocity.x = self.saturate(self.ibvs_x_inner, self.u_max_inner, -self.u_max_inner) + self.target_VE*np.cos(self.psi) - self.target_VN*np.sin(self.psi)
                ibvs_command_msg.velocity.y = self.saturate(self.ibvs_y_inner, self.v_max_inner, -self.v_max_inner) + self.target_VN*np.cos(self.psi) + self.target_VE*np.sin(self.psi)
                ibvs_command_msg.velocity.z = self.saturate(self.ibvs_F_inner, self.w_max_inner, -self.w_max_inner)

                ibvs_command_msg.yaw_rate = self.ibvs_z_inner

                # Publish.
                self.command_pub_mavros.publish(ibvs_command_msg)

            elif flag == 'outer':

                ibvs_command_msg = PositionTarget()
                ibvs_command_msg.header.stamp = rospy.get_rostime()
                ibvs_command_msg.coordinate_frame = ibvs_command_msg.FRAME_BODY_NED
                ibvs_command_msg.type_mask = self.velocity_mask

                ibvs_command_msg.velocity.x = self.saturate(self.ibvs_x, self.u_max, -self.u_max) + self.target_VE*np.cos(self.psi) - self.target_VN*np.sin(self.psi)
                ibvs_command_msg.velocity.y = self.saturate(self.ibvs_y, self.v_max, -self.v_max) + self.target_VN*np.cos(self.psi) + self.target_VE*np.sin(self.psi)
                ibvs_command_msg.velocity.z = self.saturate(self.ibvs_F, self.w_max, -self.w_max)

                ibvs_command_msg.yaw_rate = self.ibvs_z

                # Publish.
                self.command_pub_mavros.publish(ibvs_command_msg)

            else:
                pass

        else:
            if flag == 'inner':

                ibvs_command_msg = Command()
                ibvs_command_msg.x = self.saturate(self.ibvs_x_inner, self.u_max_inner, -self.u_max_inner) + self.target_VN*np.cos(self.psi) + self.target_VE*np.sin(self.psi)
                ibvs_command_msg.y = self.saturate(self.ibvs_y_inner, self.v_max_inner, -self.v_max_inner) + self.target_VE*np.cos(self.psi) - self.target_VN*np.sin(self.psi)
                ibvs_command_msg.F = self.saturate(self.ibvs_F_inner, self.w_max_inner, -self.w_max_inner)
                ibvs_command_msg.z = self.ibvs_z_inner
                ibvs_command_msg.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
                self.command_pub_roscopter.publish(ibvs_command_msg)

            elif flag == 'outer':

                ibvs_command_msg = Command()
                ibvs_command_msg.x = self.saturate(self.ibvs_x, self.u_max, -self.u_max) + self.target_VN*np.cos(self.psi) + self.target_VE*np.sin(self.psi)
                ibvs_command_msg.y = self.saturate(self.ibvs_y, self.v_max, -self.v_max) + self.target_VE*np.cos(self.psi) - self.target_VN*np.sin(self.psi)
                ibvs_command_msg.F = self.saturate(self.ibvs_F, self.w_max, -self.w_max)
                ibvs_command_msg.z = self.ibvs_z
                ibvs_command_msg.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
                self.command_pub_roscopter.publish(ibvs_command_msg)

            else:
                pass


    def send_waypoint_command(self):

        if self.mode_flag == 'mavros':
            waypoint_command_msg = PositionTarget()
            waypoint_command_msg.header.stamp = rospy.get_rostime()
            waypoint_command_msg.coordinate_frame = waypoint_command_msg.FRAME_LOCAL_NED
            waypoint_command_msg.type_mask = self.ned_pos_mask

            waypoint_command_msg.position.x = self.wp_E  # E
            waypoint_command_msg.position.y = self.wp_N  # N
            waypoint_command_msg.position.z = self.rendezvous_height  # U

            # This converts an NED heading command into an ENU heading command
            yaw = -self.heading_command + np.radians(90.0)
            while yaw > np.radians(180.0):  yaw = yaw - np.radians(360.0)
            while yaw < np.radians(-180.0):  yaw = yaw + np.radians(360.0)

            waypoint_command_msg.yaw = yaw

            # Publish.
            self.command_pub_mavros.publish(waypoint_command_msg)

        else:
            wp_command_msg = Command()
            wp_command_msg.x = self.wp_N
            wp_command_msg.y = self.wp_E
            wp_command_msg.F = -self.rendezvous_height
            wp_command_msg.z = self.heading_command
            wp_command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE

            # Publish.
            self.command_pub_roscopter.publish(wp_command_msg)


    def send_attitude_command(self):

        command_msg = AttitudeTarget()
        command_msg.header.stamp = rospy.get_rostime()
        command_msg.type_mask = self.attitude_mask

        command_msg.body_rate.x = 0.0
        command_msg.body_rate.y = 0.0
        command_msg.body_rate.z = 0.0

        command_msg.thrust = self.thrust_val

        # Publish.
        self.attitude_pub_mavros.publish(command_msg)


    def send_avg_attitude(self, event):

        # Fill out the message
        avg_attitude_msg = Point()
        avg_attitude_msg.x = self.roll_avg
        avg_attitude_msg.y = self.pitch_avg
        avg_attitude_msg.z = self.psi

        self.avg_attitude_pub.publish(avg_attitude_msg)


    def compute_rendezvous_offset(self, phi, theta):

        # Flat-earth geolocation to compute the rendezvous offset
        sphi = np.sin(phi)
        cphi = np.cos(phi)
        stheta = np.sin(theta)
        ctheta = np.cos(theta)
        spsi = np.sin(self.psi)
        cpsi = np.cos(self.psi)

        # mounting angle offsets
        phi_m = 0.0    # roll relative to the body frame
        theta_m = 0.0  # pitch '...'
        psi_m = 0.0    # yaw '...'

        ## define fixed rotations
        sphi_m = np.sin(phi_m)
        cphi_m = np.cos(phi_m)
        stheta_m = np.sin(theta_m)
        ctheta_m = np.cos(theta_m)
        spsi_m = np.sin(psi_m )
        cpsi_m = np.cos(psi_m )

        # Define rotations.
        # R_b_i = R_i_b transpose
        R_b_i = np.array([[ctheta*cpsi, ctheta*spsi, -stheta],
                          [sphi*stheta*cpsi - cphi*spsi, sphi*stheta*spsi + cphi*cpsi, sphi*ctheta],
                          [cphi*stheta*cpsi + sphi*spsi, cphi*stheta*spsi - sphi*cpsi, cphi*ctheta]]).T
        R_m_b = np.array([[ctheta_m*cpsi_m, ctheta_m*spsi_m, -stheta_m],
                          [sphi_m*stheta_m*cpsi_m-cphi_m*spsi_m, sphi_m*stheta_m*spsi_m+cphi_m*cpsi_m, sphi_m*ctheta_m],
                          [cphi_m*stheta_m*cpsi_m+sphi_m*spsi_m, cphi_m*stheta_m*spsi_m-sphi_m*cpsi_m, cphi_m*ctheta_m]]).T
        R_c_m = np.array([[0., 1., 0.],
                          [-1., 0., 0.],
                          [0., 0., 1.]]).T

        R_c_i = R_b_i.dot(R_m_b.dot(R_c_m))

        # From eq 13.9 in UAV book but we want the target at the center of the image frame and thus the unit
        # vector el_hat_c is simply:
        el_hat_c = np.array([[0.0],
                             [0.0],
                             [1.0]])
        # Vector aligned with inertial k (down) axis
        k_i = np.array([[0.0],
                        [0.0],
                        [1.0]])

        # Derrived from EQ 13.18
        numerator = R_c_i.dot(el_hat_c)
        denominator = np.dot(k_i.T, numerator)

        P_target = np.array([[0.0],
                             [0.0],
                             [0.0]])

        # EQ 13.18 rearranged
        P_uav = P_target - self.rendezvous_height*numerator/denominator

        return P_uav


    def ibvs_velocity_cmd_callback(self, msg):

        if self.mode_flag == 'mavros':
            # fill out the NED velocity vector
            self.ned_vel_vec_outer[0][0] = msg.linear.x
            self.ned_vel_vec_outer[1][0] = msg.linear.y
            self.ned_vel_vec_outer[2][0] = msg.linear.z

            # rotate into the enu frame
            enu_vec = self.ned_to_enu(self.ned_vel_vec_outer)
        
            # update state variables with the ENU data
            self.ibvs_x = enu_vec[0][0]
            self.ibvs_y = enu_vec[1][0]
            self.ibvs_F = enu_vec[2][0]
        
            # here we just flip the sign for yawrate to work with mavros FLU coord frame
            self.ibvs_z = -msg.angular.z

        else:
            # just pull off the message data
            self.ibvs_x = msg.linear.x
            self.ibvs_y = msg.linear.y
            self.ibvs_F = msg.linear.z
            self.ibvs_z = msg.angular.z

        # print '\nx_vel:', self.ibvs_x, '\ny_vel:', self.ibvs_y, '\nz_vel:', self.ibvs_F

        # Make sure we've seen the marker at least 5 times within the last second
        # get the time
        self.ibvs_time_outer = rospy.get_time()
        self.ibvs_time_queue_outer.appendleft(self.ibvs_time_outer)


    def ibvs_velocity_cmd_inner_callback(self, msg):

        if self.mode_flag == 'mavros':
            # fill out the NED velocity vector
            self.ned_vel_vec_inner[0][0] = msg.linear.x
            self.ned_vel_vec_inner[1][0] = msg.linear.y
            self.ned_vel_vec_inner[2][0] = msg.linear.z

            # rotate into the enu frame
            enu_vec = self.ned_to_enu(self.ned_vel_vec_inner)

            # update state variables with the ENU data
            self.ibvs_x_inner = enu_vec[0][0]
            self.ibvs_y_inner = enu_vec[1][0]
            self.ibvs_F_inner = enu_vec[2][0]

            # here we just flip the sign for yawrate to work with mavros FLU coord frame
            self.ibvs_z_inner = -msg.angular.z

        else:
            # just pull off the message data
            self.ibvs_x_inner = msg.linear.x
            # print "vel_x: " + str(msg.linear.x)
            self.ibvs_y_inner = msg.linear.y
            self.ibvs_F_inner = msg.linear.z
            # print "vel_down: " + str(msg.linear.z)
            self.ibvs_z_inner = msg.angular.z


        # Make sure we've seen the marker at least 5 times within the last second
        # get the time
        self.ibvs_time_inner = rospy.get_time()
        self.ibvs_time_queue_inner.appendleft(self.ibvs_time_inner)


    def ibvs_ave_error_callback(self, msg):

        self.p_des_error_outer = msg.data

    def ibvs_ave_error_inner_callback(self, msg):
        self.p_des_error_inner = msg.data


    def aruco_inner_distance_callback(self, msg):

        self.distance = msg.data
        # print(self.distance)


    def aruco_relative_heading_callback(self, msg):

        self.relative_heading = msg.data
        # print(self.distance)

    def aruco_att_callback(self, msg):

        self.R_aruco = self.get_R_from_quaternion(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z)

        vec = np.dot(self.R_aruco, np.array([[0], [0], [-1]]))

        # Find the angle between vec and the inertial k-axis.
        angle = np.arccos(np.dot(vec.T, np.array([[0], [0], [-1]])))

        # The ArUco frame is out of alignment with the camera frame by 180 degrees
        angle = np.pi - angle
        # print(np.degrees(angle))

        if self.inner_error_condition:
            if angle <= self.max_boat_angle and self.p_des_error_inner <= self.p_des_error_inner_threshold:
                self.safe_to_land = True
            else:
                self.safe_to_land = False
        else:
            if angle <= self.max_boat_angle:
                self.safe_to_land = True
            else:
                self.safe_to_land = False


    def target_callback(self, msg):

        self.target_N = msg.pose.pose.position.x
        self.target_E = msg.pose.pose.position.y

        self.wp_N = self.target_N + self.wind_offset[0][0]
        self.wp_E = self.target_E + self.wind_offset[1][0]


    def state_callback(self, msg):

        # this should already be coming in NED
        self.pn = msg.pose.pose.position.x
        self.pe = msg.pose.pose.position.y
        self.pd = msg.pose.pose.position.z

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

        # update wp_error
        # self.wp_error = np.sqrt((self.pn - self.wp_N)**2 + (self.pe - self.wp_E)**2
            # + (-self.pd - self.rendezvous_height)**2)

        # update our attitude queues
        self.phi_queue.appendleft(self.phi)
        self.theta_queue.appendleft(self.theta)


    def target_velocity_callback(self, msg):

        # Pull of the target velocity data
        self.target_VN = msg.x
        self.target_VE = msg.y


    def update_wp_error(self):

        self.wp_error = np.sqrt((self.pn - self.wp_N)**2 + (self.pe - self.wp_E)**2
            + (-self.pd - self.rendezvous_height)**2)




    def saturate(self, value, up_limit, low_limit):
        if(value > up_limit):
            rVal = up_limit
        elif(value < low_limit):
            rVal = low_limit
        else:
            rVal = value
        if (up_limit == None) or (low_limit == None):
            rVal = value

        return rVal


    def ned_to_enu(self, ned_vec):

        enu_vec = np.dot(self.R_ned_enu, ned_vec)
        return enu_vec


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


    def write_ave_att_file(self, roll_ave, pitch_ave):

        if self.mode_flag == 'mavros':

            try:
                path = '/home/nvidia/' + self.test_name + '_att_ave.txt'
                text_file = open(path, 'w')
                text_file.write('Average Roll (deg): %s' % str(np.degrees(roll_ave)))
                text_file.write('\n')
                text_file.write('Average Pitch (deg): %s' % str(np.degrees(pitch_ave)))
                text_file.close()
            except:
                print "State Machine: Error saving average attitude file."
        else:
            pass



def main():
    # initialize a node
    rospy.init_node('state_machine')

    # create instance of StateMachine class
    state_machine = StateMachine()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Shutting down')

if __name__ == '__main__':
    main()
