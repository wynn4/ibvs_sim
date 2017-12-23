#!/usr/bin/env python

import numpy as np

import rospy, tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosflight_msgs.msg import Command
from roscopter_msgs.srv import AddWaypoint, RemoveWaypoint, SetWaypointsFromFile


class WaypointManager():

    def __init__(self):

        # get parameters
        try:
            self.waypoint_list = rospy.get_param('~waypoints')
        except KeyError:
            rospy.logfatal('waypoints not set')
            rospy.signal_shutdown('Parameters not set')

        # ibvs parameters
        self.ibvs_x = 0.0
        self.ibvs_y = 0.0
        self.ibvs_F = 0.0
        self.ibvs_z = 0.0

        self.ibvs_count = 0
        self.ibvs_time = rospy.get_time()
        self.ibvs_active = False

        self.descend_slowly = -10.0  # start at 5 meters above ground

        self.descend_timer = rospy.Timer(rospy.Duration(1.0), self.descend_callback)


        # how close does the MAV need to get before going to the next waypoint?
        self.threshold = rospy.get_param('~threshold', 5)
        self.cyclical_path = rospy.get_param('~cycle', True)

        self.prev_time = rospy.Time.now()

        # set up Services
        self.add_waypoint_service = rospy.Service('add_waypoint', AddWaypoint, self.addWaypointCallback)
        self.remove_waypoint_service = rospy.Service('remove_waypoint', RemoveWaypoint, self.addWaypointCallback)
        self.set_waypoint_from_file_service = rospy.Service('set_waypoints_from_file', SetWaypointsFromFile, self.addWaypointCallback)

        # Set Up Publishers and Subscribers
        self.xhat_sub_ = rospy.Subscriber('state', Odometry, self.odometryCallback, queue_size=5)
        self.ibvs_sub = rospy.Subscriber('/ibvs/vel_cmd', Twist, self.ibvs_velocity_cmd_callback, queue_size=1)
        self.waypoint_pub_ = rospy.Publisher('high_level_command', Command, queue_size=5, latch=True)

        self.current_waypoint_index = 0

        command_msg = Command()
        current_waypoint = np.array(self.waypoint_list[0])

        command_msg.x = current_waypoint[0]
        command_msg.y = current_waypoint[1]
        command_msg.F = current_waypoint[2]
        if len(current_waypoint) > 3:
            command_msg.z = current_waypoint[3]
        else:
            next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
            delta = next_point - current_waypoint
            command_msg.z = np.atan2(delta[1], delta[0])
        command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
        self.waypoint_pub_.publish(command_msg)

        while not rospy.is_shutdown():
            # wait for new messages and call the callback when they arrive
            rospy.spin()


    def addWaypointCallback(req):
        print("addwaypoints")

    def removeWaypointCallback(req):
        print("remove Waypoints")

    def setWaypointsFromFile(req):
        print("set Waypoints from File")

    def odometryCallback(self, msg):

        time_cur = rospy.get_time()

        # reset the ibvs counter if we haven't seen the ArUco for more than 1 second
        if time_cur - self.ibvs_time >= 1.0:
            self.ibvs_count = 0
            self.ibvs_active = False
            # print "ibvs counter reset..."

        # if the ArUco has been in sight for a while
        if self.ibvs_count > 100:
            
            # print "ibvs active!"
            if self.ibvs_count > 1000:
                self.ibvs_count = 101  # reset so it doesn't get too big

            self.ibvs_active = True

            ibvs_command_msg = Command()
            ibvs_command_msg.x = self.ibvs_x
            ibvs_command_msg.y = self.ibvs_y
            # ibvs_command_msg.F = msg.pose.pose.position.z + self.ibvs_F * (5.0)  #  crazy town
            ibvs_command_msg.F = self.descend_slowly  # :)
            ibvs_command_msg.z = self.ibvs_z
            ibvs_command_msg.mode = Command.MODE_XVEL_YVEL_YAWRATE_ALTITUDE
            self.waypoint_pub_.publish(ibvs_command_msg)

        # go back to following regular waypoints    
        else:

            # Get error between waypoint and current state
            current_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
            (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
            current_position = np.array([msg.pose.pose.position.x,
                                         msg.pose.pose.position.y,
                                         msg.pose.pose.position.z])

            error = np.linalg.norm(current_position - current_waypoint[0:3])

            if error < self.threshold:
                # Get new waypoint index
                self.current_waypoint_index += 1
                if self.cyclical_path:
                    self.current_waypoint_index %= len(self.waypoint_list)
                else:
                    if self.current_waypoint_index > len(self.waypoint_list):
                        self.current_waypoint_index -=1
                next_waypoint = np.array(self.waypoint_list[self.current_waypoint_index])
                command_msg = Command()
                command_msg.x = next_waypoint[0]
                command_msg.y = next_waypoint[1]
                command_msg.F = next_waypoint[2]
                if len(current_waypoint) > 3:
                    command_msg.z = current_waypoint[3]
                else:
                    next_point = self.waypoint_list[(self.current_waypoint_index + 1) % len(self.waypoint_list)]
                    delta = next_point - current_waypoint
                    command_msg.z = np.atan2(delta[1], delta[0])
                command_msg.mode = Command.MODE_XPOS_YPOS_YAW_ALTITUDE
                self.waypoint_pub_.publish(command_msg)
                # print "waypoint published"


    def ibvs_velocity_cmd_callback(self, msg):

        # pull off the message data
        self.ibvs_x = msg.linear.x
        self.ibvs_y = msg.linear.y
        self.ibvs_F = msg.linear.z
        self.ibvs_z = msg.angular.z

        # increment the counter
        self.ibvs_count += 1

        # get the time
        self.ibvs_time = rospy.get_time()


    def descend_callback(self, event):

        if self.ibvs_active:
            self.descend_slowly += 0.1
            print(self.descend_slowly)


if __name__ == '__main__':
    rospy.init_node('waypoint_manager', anonymous=True)
    try:
        wp_manager = WaypointManager()
    except:
        rospy.ROSInterruptException
    pass
