#!/usr/bin/env python

## Simple state machine implemented in ROS that manages whether a PX4 copter
## follows position commands or IBVS velocity commands.
## JSW Mar 2018

import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
import numpy as np



# TODO:



class StateMachine():

    def __init__(self):

        # load parameters from param server
        self.u_max = rospy.get_param('~u_max', 0.5)
        self.v_max = rospy.get_param('~v_max', 0.5)
        self.w_max = rospy.get_param('~w_max', 0.7)

        self.u_max_inner = rospy.get_param('~u_max_inner', 0.2)
        self.v_max_inner = rospy.get_param('~v_max_inner', 0.2)
        self.w_max_inner = rospy.get_param('~w_max_inner', 0.2)

        self.wp_N = rospy.get_param('~wp_N', 10.0)
        self.wp_E = rospy.get_param('~wp_E', 0.0)
        self.wp_D = rospy.get_param('~wp_D', -15.0)


        # ibvs parameters
        # outer
        self.ibvs_x = 0.0
        self.ibvs_y = 0.0
        self.ibvs_F = 0.0
        self.ibvs_z = 0.0

        self.ibvs_count = 0
        self.ibvs_time = rospy.get_time() - 10.0

        # inner
        self.ibvs_x_inner = 0.0
        self.ibvs_y_inner = 0.0
        self.ibvs_F_inner = 0.0
        self.ibvs_z_inner = 0.0

        self.ibvs_count_inner = 0
        self.ibvs_time_inner = rospy.get_time() - 10.0

        self.counters_frozen = False
        self.distance = 10.0

        self.land_mode_sent = False

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

        # Set Up Publishers and Subscribers
        self.ibvs_sub = rospy.Subscriber('/ibvs/vel_cmd', Twist, self.ibvs_velocity_cmd_callback, queue_size=1)
        self.ibvs_inner_sub = rospy.Subscriber('/ibvs_inner/vel_cmd', Twist, self.ibvs_velocity_cmd_inner_callback, queue_size=1)
        self.aruco_sub = rospy.Subscriber('/aruco/distance_inner', Float32, self.aruco_inner_distance_callback)
        self.command_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.ibvs_active_pub_ = rospy.Publisher('ibvs_active', Bool, queue_size=1)

        # Set Up Service Proxy
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        self.ibvs_active_msg = Bool()
        self.ibvs_active_msg.data = False

        # Initialize timers.
        self.update_rate = 20.0
        self.update_timer = rospy.Timer(rospy.Duration(1.0/self.update_rate), self.send_commands)


    def send_commands(self, msg):

        time_cur = rospy.get_time()

        self.ibvs_active_pub_.publish(self.ibvs_active_msg)

        # reset the ibvs counter if we haven't seen the ArUco for more than 1 second
        if time_cur - self.ibvs_time >= 1.0 and time_cur - self.ibvs_time_inner >= 1.0 and self.counters_frozen == False:
            self.ibvs_count = 0
            self.ibvs_count_inner = 0
            self.ibvs_active_msg.data = False
            # print "reset"

        # if the ArUco has been in sight for a while
        if self.ibvs_count > 100 or self.ibvs_count_inner > 30:
            
            self.ibvs_active_msg.data = True

            if self.ibvs_count_inner > 30:

                print(self.distance)
                # in this case we enter an open-loop drop onto the marker
                if self.distance < 1.0 and self.distance > 0.1 and self.land_mode_sent == False:
                    
                    # set the PX4 to land mode
                    self.execute_landing()
                    # freeze the counters
                    # self.counters_frozen = True
                    # ibvs_command_msg = Command()
                    # ibvs_command_msg.x = 0.0
                    # ibvs_command_msg.y = 0.0
                    # ibvs_command_msg.F = 0.0
                    # ibvs_command_msg.z = 0.0
                    # ibvs_command_msg.mode = Command.MODE_ROLL_PITCH_YAWRATE_THROTTLE
                    # self.waypoint_pub_.publish(ibvs_command_msg)
                    # # stuff
                else:
                    ibvs_command_msg = PositionTarget()
                    ibvs_command_msg.header.stamp = rospy.get_rostime()
                    ibvs_command_msg.coordinate_frame = ibvs_command_msg.FRAME_BODY_NED
                    ibvs_command_msg.type_mask = self.velocity_mask

                    ibvs_command_msg.velocity.x = self.saturate(self.ibvs_x_inner, self.u_max_inner, -self.u_max_inner)
                    ibvs_command_msg.velocity.y = self.saturate(self.ibvs_y_inner, self.v_max_inner, -self.v_max_inner)
                    ibvs_command_msg.velocity.z = self.saturate(self.ibvs_F_inner, self.w_max_inner, -self.w_max_inner)

                    ibvs_command_msg.yaw_rate = self.ibvs_z_inner

                    # Publish.
                    self.command_pub.publish(ibvs_command_msg)

                    if self.ibvs_count_inner > 1000:
                        self.ibvs_count_inner = 31  # reset so it doesn't get too big
            else:
                ibvs_command_msg = PositionTarget()
                ibvs_command_msg.header.stamp = rospy.get_rostime()
                ibvs_command_msg.coordinate_frame = ibvs_command_msg.FRAME_BODY_NED
                ibvs_command_msg.type_mask = self.velocity_mask

                ibvs_command_msg.velocity.x = self.saturate(self.ibvs_x, self.u_max, -self.u_max)
                ibvs_command_msg.velocity.y = self.saturate(self.ibvs_y, self.v_max, -self.v_max)
                ibvs_command_msg.velocity.z = self.saturate(self.ibvs_F, self.w_max, -self.w_max)

                ibvs_command_msg.yaw_rate = self.ibvs_z

                # Publish.
                self.command_pub.publish(ibvs_command_msg)

                if self.ibvs_count > 1000:
                    self.ibvs_count = 101  # reset so it doesn't get too big

        # go back to following regular waypoints    
        else:
            # print "following waypoints"
            waypoint_command_msg = PositionTarget()
            waypoint_command_msg.header.stamp = rospy.get_rostime()
            waypoint_command_msg.coordinate_frame = waypoint_command_msg.FRAME_LOCAL_NED
            waypoint_command_msg.type_mask = self.ned_pos_mask

            waypoint_command_msg.position.x = self.wp_E  # E
            waypoint_command_msg.position.y = self.wp_N  # N
            waypoint_command_msg.position.z = -self.wp_D  # U

            waypoint_command_msg.yaw = np.radians(0.0)  # Point North?

            # Publish.
            self.command_pub.publish(waypoint_command_msg)


    def ibvs_velocity_cmd_callback(self, msg):

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

        # print '\nx_vel:', self.ibvs_x, '\ny_vel:', self.ibvs_y, '\nz_vel:', self.ibvs_F

        # increment the counter
        self.ibvs_count += 1

        # get the time
        self.ibvs_time = rospy.get_time()


    def ibvs_velocity_cmd_inner_callback(self, msg):

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

        # print '\nx_vel:', self.ibvs_x, '\ny_vel:', self.ibvs_y, '\nz_vel:', self.ibvs_F

        # increment the counter
        self.ibvs_count_inner += 1

        # get the time
        self.ibvs_time_inner = rospy.get_time()


    def aruco_inner_distance_callback(self, msg):

        self.distance = msg.data
        # print(self.distance)


    def execute_landing(self):

        rospy.wait_for_service('/mavros/set_mode')
        try:
            isModeChanged = self.set_mode_srv(custom_mode='AUTO.LAND')
            
            if isModeChanged:
                self.land_mode_sent = True
                print "mode %s sent." % 'AUTO.LAND'

        except rospy.ServiceException, e:
            print "service call set_mode failed: %s" % e


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
