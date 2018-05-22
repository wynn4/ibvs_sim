#! /usr/bin/env python

import rospy
from std_msgs.msg import Bool

class SaveNow(object):

	def __init__(self):

		self.publisher = rospy.Publisher('/save_ibvs_mat_now', Bool, queue_size=1)
		self.msg = Bool()
		self.msg.data = True

		# Initialize timers.
		self.rate = 1.0
		self.timer = rospy.Timer(rospy.Duration(1.0/self.rate), self.send_save)

	def send_save(self, event):
		self.publisher.publish(self.msg)
   


def main():
    # initialize a node
    rospy.init_node('save_now')

    # create instance of SaveNow class
    save = SaveNow()

    # spin
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main()
