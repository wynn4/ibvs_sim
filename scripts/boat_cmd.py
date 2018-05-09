#!/usr/bin/env python

# import sys
import rospy
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Joy
import math

LINEAR_MULTIPLIER = 2.0
ANGULAR_MULTIPLIER = 0.2

class move:

    def __init__(self):

        self.boat_pub = rospy.Publisher('boat_command', Pose, queue_size=1)
        #self.joy_sub = rospy.Subscriber('/joy_throttled', Joy, self.joy_callback, queue_size=1) # toggle in yaml file

        self.cmd = Pose()
        self.cmd.position.x = rospy.get_param('~x_pos_cmd', 0.0)  # desired x position
        self.cmd.position.y = rospy.get_param('~y_pos_cmd', 0.0)  # desired y position
        self.cmd.position.z = 0.0  # desired z altitude
        self.cmd.orientation.x = 0.0  # desired roll angle
        self.cmd.orientation.y = 0.0  # desired pitch angle
        self.cmd.orientation.z = 0.0  # desired yaw angle
        # Will go unused, for now (not using quaternion notation)
        self.cmd.orientation.w = 0.0

        self.max_angle = 25.0 * math.pi / 180.0

        self.update_rate = 10.0
        self.update_timer_ = rospy.Timer(
            rospy.Duration(1.0 / self.update_rate), self.update)

        self.start_time = rospy.get_time()
        self.current_time = self.start_time
        self.omega = 1.0 / 5.0

        self.boat_pub.publish(self.cmd)  # ----------------?

    def joy_callback(self, Joy):
        #print('callback called.')
        self.cmd.position.x += LINEAR_MULTIPLIER * Joy.axes[3]
        self.cmd.position.y += LINEAR_MULTIPLIER * Joy.axes[4] * -1.0
        self.cmd.position.z += LINEAR_MULTIPLIER * Joy.axes[1]
        if Joy.buttons[4] == 1:  # Pitch
            if self.cmd.orientation.y + ANGULAR_MULTIPLIER <= 1.5:
                self.cmd.orientation.y += ANGULAR_MULTIPLIER
            else:
                self.cmd.orientation.y = 1.5
        elif Joy.buttons[5] == 1:
            if self.cmd.orientation.y - ANGULAR_MULTIPLIER >= -1.5:
                self.cmd.orientation.y -= ANGULAR_MULTIPLIER
            else:
                self.cmd.orientation.y = -1.5
        if Joy.buttons[2] == 1:  # Yaw
            if self.cmd.orientation.z + ANGULAR_MULTIPLIER <= 1.5:
                self.cmd.orientation.z += ANGULAR_MULTIPLIER
            else:
                self.cmd.orientation.z = 1.5
        elif Joy.buttons[1] == 1:
            if self.cmd.orientation.z - ANGULAR_MULTIPLIER >= -1.5:
                self.cmd.orientation.z -= ANGULAR_MULTIPLIER
            else:
                self.cmd.orientation.z = -1.5
        '''
	    if Joy.buttons[2] == 1:
		    self.cmd.orientation.z += ANGULAR_MULTIPLIER
	    elif Joy.buttons[1] == 1:
		    self.cmd.orientation.z -= ANGULAR_MULTIPLIER
	    '''
        if Joy.buttons[0] == 1:  # Roll
            if self.cmd.orientation.x + ANGULAR_MULTIPLIER <= 1.5:
                self.cmd.orientation.x += ANGULAR_MULTIPLIER
            else:
                self.cmd.orientation.x = 1.5
        elif Joy.buttons[3] == 1:
            if self.cmd.orientation.x - ANGULAR_MULTIPLIER >= -1.5:
                self.cmd.orientation.x -= ANGULAR_MULTIPLIER
            else:
                self.cmd.orientation.x = -1.5
        self.boat_pub.publish(self.cmd)

    def update(self, event):
        self.current_time = rospy.get_time()
        time = self.current_time - self.start_time

        self.cmd.position.z = 0.5 * math.sin(0.2 * time)
        self.cmd.orientation.x = self.max_angle * math.sin(0.5 * time)
        self.cmd.orientation.y = self.max_angle * math.sin(0.6 * time)
        
        self.boat_pub.publish(self.cmd)

if __name__ == "__main__":

    rospy.init_node('boat_cmd', anonymous=True)
    boat = move()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down Boat Command Node.'
