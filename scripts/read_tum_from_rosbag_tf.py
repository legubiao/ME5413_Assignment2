#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import os
import rosbag
import argparse

import tf
import tf.transformations as tft
import geometry_msgs.msg

class OdomToTUM:
    def __init__(self, bag_file, topic_name, source_frame, target_frame):
        self.file_path = 'odom_tum.txt'
        if os.path.exists(self.file_path):
            os.remove(self.file_path)
        self.file = open(self.file_path, 'w')
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.first_timestamp = 1317354879.24
        self.process_bag(bag_file, topic_name)

    def process_bag(self, bag_file, topic_name):
        bag = rosbag.Bag(bag_file)
        for topic, msg, t in bag.read_messages(topics=['/tf_static']):
            for trans in msg.transforms:
                if trans.child_frame_id == self.target_frame and trans.header.frame_id == self.source_frame:
                    self.transform = trans
                    break
        for topic, msg, t in bag.read_messages(topics=[topic_name]):
            self.callback(msg)
        bag.close()

    def callback(self, msg):
        trans = self.transform.transform.translation
        rot = self.transform.transform.rotation
        pose = geometry_msgs.msg.PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        pose.pose.position.x -= trans.x
        pose.pose.position.y -= trans.y
        pose.pose.position.z -= trans.z
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        )
        euler = tft.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
        q_rot = tft.quaternion_from_euler(euler[0], euler[1], euler[2])
        q_pose = tft.quaternion_multiply(quaternion, q_rot)
        pose.pose.orientation = geometry_msgs.msg.Quaternion(*q_pose)

        time = msg.header.stamp.to_sec() - self.first_timestamp
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w
        )
        self.file.write(f"{time} {-pose.pose.position.y} {pose.pose.position.z} {pose.pose.position.x} {quaternion[0]} {quaternion[1]} {quaternion[2]} {quaternion[3]}\n")

    def run(self):
        self.file.close()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process a rosbag file and convert /odom messages to TUM format.')
    parser.add_argument('bag_file', help='Input rosbag file')
    parser.add_argument('--topic', default='/odom', help='Topic to subscribe to')
    parser.add_argument('--source_frame', default='imu_link', help='Source tf frame')
    parser.add_argument('--target_frame', default='camera_gray_left', help='Target tf frame')
    args = parser.parse_args()

    try:
        node = OdomToTUM(args.bag_file, args.topic, args.source_frame, args.target_frame)
        node.run()
    except rospy.ROSInterruptException:
        pass
