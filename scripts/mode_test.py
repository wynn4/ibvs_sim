#! /usr/bin/env python

## ROS node.
## JSW Mar 2018

import rospy
from mavros_msgs.srv import SetMode
import numpy as np


class ModeTest(object):

    def __init__(self):

        # Load ROS params.
        self.mode_str = 'AUTO.LAND'
        
        self.set_mode_srv = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        
        self.count_rate = 1.0/5.0  # ever 5 seconds
        self.count_timer = rospy.Timer(rospy.Duration(1.0/self.count_rate), self.flip_modes)



    def flip_modes(self, event):

        # mode_switch = SetMode
        # mode_switch.custom_mode = self.mode_str
        # mode_switch.mode_sent = True
        rospy.wait_for_service('/mavros/set_mode')
        try:
            isModeChanged = self.set_mode_srv(custom_mode=self.mode_str)
            
            if isModeChanged:
                print "mode %s sent." % self.mode_str

        except rospy.ServiceException, e:
            print "service call set_mode failed: %s" %e

        if self.mode_str == 'MANUAL':
            self.mode_str = 'STABILIZED'
        elif self.mode_str == 'STABILIZED':
            self.mode_str = 'MANUAL'
        else:
            pass

        # print "mode sent"
        

        



def main():
    # initialize a node
    rospy.init_node('test_mavros_modes')

    # create instance of ModeTest class
    test = ModeTest()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
