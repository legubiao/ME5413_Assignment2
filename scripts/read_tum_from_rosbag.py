#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import os
import rosbag
import argparse

class OdomToTUM:
    def __init__(self, bag_file, topic_name):
        self.file_path = 'odom_tum.txt'
        if os.path.exists(self.file_path):
            os.remove(self.file_path)
        self.file = open(self.file_path, 'w')
        self.first_timestamp = 1317354879.24
        self.process_bag(bag_file, topic_name)

    def process_bag(self, bag_file, topic_name):
        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            self.callback(msg)
        bag.close()

    def callback(self, msg):
        time = msg.header.stamp.to_sec() - self.first_timestamp
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )
        self.file.write(f"{time} {-msg.pose.pose.position.y} {msg.pose.pose.position.z} {msg.pose.pose.position.x} {quaternion[0]} {quaternion[1]} {quaternion[2]} {quaternion[3]}\n")

    def run(self):
        self.file.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process a rosbag file and convert /odom messages to TUM format.')
    parser.add_argument('bag_file', help='Input rosbag file')
    parser.add_argument('--topic', default='/odom', help='Topic to subscribe to')
    args = parser.parse_args()

    try:
        node = OdomToTUM(args.bag_file, args.topic)
        node.run()
    except rospy.ROSInterruptException:
        pass
