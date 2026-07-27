#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

class Publisher:
    def __init__(self):
        # Parameters
        self.message = rospy.get_param('~message', 'Hello World!')
        rate = rospy.get_param('~rate', 2.0)
        # Internal variables
        self.rate = rospy.Rate(rate)  # publish frequency in Hz

        # Publishers
        self.pub = rospy.Publisher('/message', String, queue_size=10)  # creates a publisher on the /message topic

    def run(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.message)
            
            self.rate.sleep()  # sleeps until it's time to publish again

if __name__ == '__main__':
    rospy.init_node('publisher')
    node = Publisher()
    node.run()