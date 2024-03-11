#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import os

class OdomToTUM:
    def __init__(self):
        rospy.init_node('odom_to_tum', anonymous=True)
        topic_name = rospy.get_param('~odom_topic', '/odom')
        self.odom_sub = rospy.Subscriber(topic_name, Odometry, self.callback)
        self.file_path = 'odom_tum.txt'
        if os.path.exists(self.file_path):
            os.remove(self.file_path)
        self.file = open(self.file_path, 'w')

    def callback(self, msg):
        self.file.write(f"{msg.header.stamp.to_sec()} {msg.pose.pose.position.x} {msg.pose.pose.position.y} {msg.pose.pose.position.z} 
{msg.pose.pose.orientation.x} {msg.pose.pose.orientation.y} {msg.pose.pose.orientation.z} {msg.pose.pose.orientation.w}\n")

    def run(self):
        rospy.spin()
        self.file.close()

if __name__ == '__main__':
    try:
        node = OdomToTUM()
        node.run()
    except rospy.ROSInterruptException:
        pass
