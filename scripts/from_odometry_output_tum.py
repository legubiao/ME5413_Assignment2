#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
import tf2_ros

counter = 0

first_timestamp = 1317354879.24

def callback(data):
    global counter
    global f
    global first_timestamp
    # Get pose from odometry
    pose = data.pose.pose
    # Retrieve position and orientation
    position = pose.position
    orientation = pose.orientation
    quaternion = (
        orientation.x,
        orientation.y,
        orientation.z,
        orientation.w
    )
    # Convert quaternion to rotation matrix
    rotation_matrix = np.array(tf.transformations.quaternion_matrix(quaternion)[0:3, 0:3])
    # Extract the translation vector
    translation_vector = np.array([position.x, position.y, position.z])
    # Get 4*4 matrix
    lidar_pose = np.zeros((4, 4))
    lidar_pose[:3, :3] = rotation_matrix
    lidar_pose[:3, 3] = translation_vector
    lidar_pose[3, 3] = 1
    # Transform the lidar pose to the camera frame
    camera_pose = np.dot(tfchange, lidar_pose)

    #Convert rotation to quaternion matrix
    camera_quaternion = tf.transformations.quaternion_from_matrix(camera_pose)
    # Get timestamp
    timestamp = data.header.stamp.to_sec() - first_timestamp
    # Format data to write output
    formatted_data = [timestamp, camera_pose[0, 3], camera_pose[1, 3], camera_pose[2, 3], camera_quaternion[0], camera_quaternion[1], camera_quaternion[2], camera_quaternion[3]]
    f.write(' '.join(["%.7e" % item for item in formatted_data]) + '\n')
    counter += 1
    sys.stdout.write("\rProcessed %i poses" % counter)
    sys.stdout.flush()


def listener():
    rospy.init_node('lego', anonymous=True)
    print("start listener")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # Wait for the transform to become available
    if not tf_buffer.can_transform('camera_gray_left', 'camera_init', rospy.Time(), timeout=rospy.Duration(5.0)):
        rospy.logerr('Failed to find transform.')
        print("Failed to find transform1")
        return

    # Lookup the transform
    try:
        transform_stamped = tf_buffer.lookup_transform('camera_gray_left', 'camera_init', rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logerr('Failed to find transform.')
        print("Failed to find transform2")
        return

    rospy.loginfo("Successfully found transform: ")
    print("success find transform")

    rospy.loginfo("Translation Vector: [%s, %s, %s]" % (transform_stamped.transform.translation.x,
                                                        transform_stamped.transform.translation.y,
                                                        transform_stamped.transform.translation.z))

    rospy.loginfo("Rotation Quaternion: [%s, %s, %s, %s]" % (transform_stamped.transform.rotation.x,
                                                             transform_stamped.transform.rotation.y,
                                                             transform_stamped.transform.rotation.z,
                                                             transform_stamped.transform.rotation.w))
    # Convert rotation from Quaternion to 4x4 matrix
    transform_matrix = tf.transformations.quaternion_matrix([transform_stamped.transform.rotation.x,
                                                             transform_stamped.transform.rotation.y,
                                                             transform_stamped.transform.rotation.z,
                                                             transform_stamped.transform.rotation.w])
    # Insert translations into 4x4 matrix
    transform_matrix[0, 3] = transform_stamped.transform.translation.x
    transform_matrix[1, 3] = transform_stamped.transform.translation.y
    transform_matrix[2, 3] = transform_stamped.transform.translation.z

    rospy.loginfo("Resulting Transform Matrix: \n %s " % transform_matrix)

    global tfchange
    tfchange = transform_matrix

    # Subscribe to Odometry topic
    rospy.Subscriber("/aft_mapped_to_init", Odometry, callback, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    global f
    # Open file to write output
    f = open("/home/yanqiao/Downloads/Task2/lego_tum.txt", "w")
    print("Start program")
    listener()
    f.close()